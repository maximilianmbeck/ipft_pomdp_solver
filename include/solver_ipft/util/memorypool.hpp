#pragma once

#include <cassert>
#include <ostream>
#include <vector>

/**
 * This piece of code is taken from POMCP solver
 *
 * cf. Silver, David, and Joel Veness. "Monte-Carlo planning in large POMDPs."
 * Neural Information Processing Systems, 2010.
 */

namespace solver_ipft {

class MemoryObject {
public:
    void SetAllocated() {
        allocated_ = true;
    }
    void ClearAllocated() {
        allocated_ = false;
    }
    bool IsAllocated() const {
        return allocated_;
    }

private:
    bool allocated_;
};

template <class T>
class MemoryPool {
public:
    MemoryPool() = default;

    ~MemoryPool() {
        DeleteAll();
    }

    MemoryPool(const MemoryPool&) = delete;
    MemoryPool(MemoryPool&&) = delete;
    MemoryPool& operator=(const MemoryPool&) = delete;
    MemoryPool& operator=(MemoryPool&&) = delete;

    //! DO NOT USE -> causes assertion failure
    T* Construct() {
        T* obj = Allocate();
        return new (obj) T;
    }
    //! DO NOT USE -> causes assertion failure
    void Destroy(T* obj) {
        obj->~T();
        Free(obj);
    }

    T* Allocate() {
        if (freelist_.empty()) {
            NewChunk();
        }
        T* obj = freelist_.back();
        freelist_.pop_back();
        assert(!obj->IsAllocated());
        obj->SetAllocated();
        num_allocated_++;
        return obj;
    }

    void Free(T* obj) {
        assert(obj->IsAllocated());
        obj->ClearAllocated();
        freelist_.push_back(obj);
        num_allocated_--;
    }

    void DeleteAll() {
        for (chunk_iterator_ i_chunk = chunks_.begin(); i_chunk != chunks_.end(); ++i_chunk) {
            delete *i_chunk;
        }
        chunks_.clear();
        freelist_.clear();
        num_allocated_ = 0;
    }

    int num_allocated() const {
        return num_allocated_;
    }

private:
    struct Chunk {
        static const int Size = 256;
        T Objects[Size];
    };

    void NewChunk() {
        Chunk* chunk = new Chunk;
        chunks_.push_back(chunk);
        for (int i = Chunk::Size - 1; i >= 0; --i) {
            freelist_.push_back(&chunk->Objects[i]);
            chunk->Objects[i].ClearAllocated();
        }
    }

    std::vector<Chunk*> chunks_;
    std::vector<T*> freelist_;
    using chunk_iterator_ = typename std::vector<Chunk*>::iterator;

public:
    int num_allocated_{0};
};

} // namespace solver_ipft
