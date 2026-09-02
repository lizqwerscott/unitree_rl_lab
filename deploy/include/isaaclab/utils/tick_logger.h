// Per-policy-tick recorder for hardware runs.
//
// Nothing a bag can record tells you what the policy actually consumed: the depth frame the
// encoder is handed is chosen from a ring buffer, the encoder latent and the GRU hidden state are
// never published, and there is no action topic at all. Without those, "the policy emitted a bad
// command" cannot be separated from "the command was fine and the joint failed to track it".
//
// Design constraints, in order:
//
//  1. The 50 Hz control thread must never block. It only memcpys into a preallocated slab and
//     bumps an atomic counter; a background thread does every file write.
//  2. Nothing is allocated after construction. If the writer somehow falls behind by more than the
//     slab holds, records are dropped and the loss is flagged in the file rather than papered over.
//  3. The file is self-describing. Field names, offsets, counts and dtypes are written into the
//     header, so a schema change cannot silently misalign the parser.
//
// At the parkour schema a record is ~4.8 kB, so 50 Hz costs ~240 kB/s and the default 2000-record
// slab buffers 40 s of writer stall.

#pragma once

#include <atomic>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <chrono>
#include <thread>
#include <vector>

namespace isaaclab
{

class TickLogger
{
public:
    enum DType : uint8_t { F32 = 0, F64 = 1, U64 = 2 };

    struct Field
    {
        std::string name;
        uint32_t count;
        DType dtype;
    };

    static constexpr char MAGIC[8] = {'P', 'K', 'R', 'T', 'I', 'C', 'K', '\1'};
    static constexpr uint32_t NAME_BYTES = 24;

    TickLogger() = default;

    /// Fix the schema and reserve the slab. Field order is the record layout.
    void configure(std::vector<Field> schema, size_t capacity_records = 2000)
    {
        _schema = std::move(schema);
        _record_bytes = 0;
        for (const auto& f : _schema) {
            _record_bytes += f.count * (f.dtype == F32 ? 4u : 8u);
        }
        _capacity = capacity_records ? capacity_records : 1;
        _slab.assign(_record_bytes * _capacity, 0);
        _count.store(0);
        _flushed = 0;
        _dropped.store(0);
        _configured = _record_bytes > 0;
    }

    bool configured() const { return _configured; }
    size_t record_bytes() const { return _record_bytes; }
    uint64_t recorded() const { return _count.load(std::memory_order_acquire); }
    uint64_t dropped() const { return _dropped.load(std::memory_order_relaxed); }

    /// Open the file, write the header, and start the writer thread.
    bool start(const std::string& path)
    {
        if (!_configured || _running.load()) return false;
        _file = std::fopen(path.c_str(), "wb");
        if (!_file) return false;
        _path = path;
        write_header();
        _running.store(true);
        _writer = std::thread([this] { writer_loop(); });
        return true;
    }

    /// Claim the next record slot. Returns nullptr when the slab has overrun (record dropped).
    /// Control-thread only. Fill the returned bytes, then call commit().
    uint8_t* begin_record()
    {
        if (!_running.load(std::memory_order_relaxed)) return nullptr;
        const uint64_t written = _count.load(std::memory_order_relaxed);
        if (written - _flushed_seen.load(std::memory_order_acquire) >= _capacity) {
            _dropped.fetch_add(1, std::memory_order_relaxed);
            return nullptr;
        }
        return _slab.data() + (written % _capacity) * _record_bytes;
    }

    /// Publish the record filled by begin_record(). Control-thread only.
    void commit() { _count.fetch_add(1, std::memory_order_release); }

    /// Flush the tail and close. Safe to call more than once.
    void stop()
    {
        if (!_running.load()) return;
        _running.store(false);
        if (_writer.joinable()) _writer.join();
        if (_file) {
            drain();
            patch_trailer();
            std::fclose(_file);
            _file = nullptr;
        }
        std::printf("[tick_logger] wrote %llu records (%llu dropped) to %s\n",
                    (unsigned long long)_count.load(), (unsigned long long)_dropped.load(),
                    _path.c_str());
    }

    ~TickLogger() { stop(); }

private:
    void write_header()
    {
        std::fwrite(MAGIC, 1, sizeof(MAGIC), _file);
        const uint32_t record_bytes = static_cast<uint32_t>(_record_bytes);
        const uint32_t field_count = static_cast<uint32_t>(_schema.size());
        std::fwrite(&record_bytes, 4, 1, _file);
        std::fwrite(&field_count, 4, 1, _file);
        uint32_t offset = 0;
        for (const auto& f : _schema) {
            char name[NAME_BYTES] = {0};
            std::strncpy(name, f.name.c_str(), NAME_BYTES - 1);
            const uint8_t dtype = static_cast<uint8_t>(f.dtype);
            std::fwrite(name, 1, NAME_BYTES, _file);
            std::fwrite(&offset, 4, 1, _file);
            std::fwrite(&f.count, 4, 1, _file);
            std::fwrite(&dtype, 1, 1, _file);
            offset += f.count * (f.dtype == F32 ? 4u : 8u);
        }
        // Record count and drop count are unknown until the end; reserve and patch on close.
        _trailer_pos = std::ftell(_file);
        const uint64_t placeholder = 0;
        std::fwrite(&placeholder, 8, 1, _file);   // records
        std::fwrite(&placeholder, 8, 1, _file);   // dropped
        std::fflush(_file);
    }

    void patch_trailer()
    {
        const uint64_t records = _flushed;
        const uint64_t dropped = _dropped.load();
        std::fseek(_file, _trailer_pos, SEEK_SET);
        std::fwrite(&records, 8, 1, _file);
        std::fwrite(&dropped, 8, 1, _file);
        std::fflush(_file);
    }

    /// Append every record the control thread has published since the last call.
    void drain()
    {
        const uint64_t target = _count.load(std::memory_order_acquire);
        while (_flushed < target) {
            const size_t slot = static_cast<size_t>(_flushed % _capacity);
            std::fwrite(_slab.data() + slot * _record_bytes, 1, _record_bytes, _file);
            ++_flushed;
        }
        _flushed_seen.store(_flushed, std::memory_order_release);
    }

    void writer_loop()
    {
        // Poll well inside one 50 Hz control period so the slab stays nearly empty and a brief
        // disk stall, not a slow drain, is the only way to lose a record.
        while (_running.load(std::memory_order_relaxed)) {
            drain();
            std::fflush(_file);
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
    }

    std::vector<Field> _schema;
    std::vector<uint8_t> _slab;
    size_t _record_bytes = 0;
    size_t _capacity = 0;
    bool _configured = false;

    std::atomic<uint64_t> _count{0};        // published by the control thread
    std::atomic<uint64_t> _flushed_seen{0}; // published by the writer, read by the control thread
    std::atomic<uint64_t> _dropped{0};
    uint64_t _flushed = 0;                  // writer thread only

    std::FILE* _file = nullptr;
    std::string _path;
    long _trailer_pos = 0;
    std::atomic<bool> _running{false};
    std::thread _writer;
};

/// Small cursor so a call site fills a record positionally without recomputing offsets.
class RecordWriter
{
public:
    explicit RecordWriter(uint8_t* base) : _p(base) {}

    void f64(double v) { std::memcpy(_p, &v, 8); _p += 8; }
    void u64(uint64_t v) { std::memcpy(_p, &v, 8); _p += 8; }
    void f32(float v) { std::memcpy(_p, &v, 4); _p += 4; }

    /// Copy `n` floats, zero-padding if the source is short so the layout never shifts.
    void f32n(const float* src, size_t n, size_t have)
    {
        const size_t copy = have < n ? have : n;
        if (copy) std::memcpy(_p, src, copy * 4);
        if (copy < n) std::memset(_p + copy * 4, 0, (n - copy) * 4);
        _p += n * 4;
    }

    void f32n(const std::vector<float>& v, size_t n) { f32n(v.data(), n, v.size()); }

private:
    uint8_t* _p;
};

}  // namespace isaaclab
