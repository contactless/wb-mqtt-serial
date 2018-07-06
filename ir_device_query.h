#pragma once

#include "declarations.h"
#include "utils.h"
#include "types.h"
#include "ir_device_memory_view.h"

#include <list>
#include <vector>
#include <cassert>

/**
 * Intermediate, protocol-agnostic device query representation format data structures
 */
struct TIRDeviceQuery
{
    friend class TIRDeviceQueryFactory;

    const TPSetRange<PMemoryBlock>  MemoryBlockRange;
    const std::vector<PVirtualRegister>   VirtualRegisters;    // registers that will be fully read or written after execution of query
    const bool                      HasHoles;
    const EQueryOperation           Operation;

private:
    mutable EQueryStatus Status;
    bool                 AbleToSplit;

protected:
    /**
     * @brief create query with binding to virtual registers.
     *  It'll update virtual registers values on finalize and
     *  maintain memory blocks cache in correct state as side effect.
     */
    explicit TIRDeviceQuery(TAssociatedMemoryBlockSet &&, EQueryOperation = EQueryOperation::Read);

    /**
     * @brief create query without binding to virtual registers.
     *  It'll only maintain memory blocks cache in correct state as side effect.
     *
     * @note needed for setup sections. Setup sections should not interfere with
     *  main polling objects (TSerialClient) by creating it's own virtual register,
     *  thus we provide here option which doesn't require virtual register existence.
     */
    explicit TIRDeviceQuery(const TPSet<PMemoryBlock> &, EQueryOperation = EQueryOperation::Read);

    void SetStatus(EQueryStatus) const;

    template <typename T>
    static void CheckTypeSingle()
    {
        static_assert(std::is_fundamental<T>::value, "only fundamental types are allowed");
        static_assert(sizeof(T) <= sizeof(uint64_t), "size of type exceeded 64 bits");
    };

    template <typename T>
    static void CheckTypeMany()
    {
        CheckTypeSingle<T>();
        static_assert(!std::is_same<T, bool>::value, "vector<bool> is not supported");
    };

public:
    virtual ~TIRDeviceQuery() = default;

    bool operator<(const TIRDeviceQuery &) const noexcept;

    PSerialDevice GetDevice() const;
    uint32_t GetBlockCount() const;
    uint32_t GetValueCount() const;
    uint32_t GetStart() const;
    uint16_t GetBlockSize() const;
    uint32_t GetSize() const;
    const TMemoryBlockType & GetType() const;
    const std::string & GetTypeName() const;

    /**
     * @brief set status of query execution
     *
     * @note this is set by device after execution
     *  of query and on exceptions during query execution.
     */
    void SetStatus(EQueryStatus);
    EQueryStatus GetStatus() const;
    void ResetStatus();
    void InvalidateReadValues();

    /**
     * @brief used to set enabled all affected virtual registers
     */
    void SetEnabledWithRegisters(bool);
    /**
     * @brief returns true if there's any enabled virtual register
     *  affected by this query
     */
    bool IsEnabled() const;
    /**
     * @brief returns true if query was executed by device,
     *  successfully or not
     */
    bool IsExecuted() const;
    /**
     * @brief indicates ability of this query to split into
     *  multiple lesser queries
     */
    bool IsAbleToSplit() const;
    /**
     * @brief used to set ability to split externally
     *
     * @note we cannot say for sure wether or not we able to split
     *  query because split for some reasons might end up with error
     *  or with only one query, so in that case we manually mark that
     *  query as not able to split.
     */
    void SetAbleToSplit(bool);

    /**
     * @brief create view to passed memory according to query's data layout
     */
    TIRDeviceMemoryView CreateMemoryView(void * mem, size_t size) const;
    TIRDeviceMemoryView CreateMemoryView(const void * mem, size_t size) const;

    template <class T>
    const T & As() const
    {
        static_assert(std::is_base_of<TIRDeviceQuery, T>::value, "trying to cast to type not derived from TIRDeviceQuery");

        const T * pointer = dynamic_cast<const T *>(this);
        assert(pointer);
        return *pointer;
    }

    /**
     * Accept read memory from device as current and set status to Ok
     */
    void FinalizeRead(const void * mem, size_t size) const
    {
        FinalizeRead(CreateMemoryView(mem, size));
    }

    /**
     * Accept read memory from device as current and set status to Ok (dynamic array)
     */
    template <typename T>
    void FinalizeRead(const std::vector<T> & mem) const
    {
        CheckTypeMany<T>();

        FinalizeRead(CreateMemoryView(mem.data(), sizeof(T) * mem.size()));
    }

    /**
     * Accept read memory from device as current and set status to Ok (static array)
     */
    template <typename T, size_t N>
    void FinalizeRead(const T (& mem)[N]) const
    {
        CheckTypeMany<T>();

        FinalizeRead(CreateMemoryView(mem, sizeof(T) * N));
    }

    virtual void FinalizeRead(const TIRDeviceMemoryView &) const;

    std::string Describe() const;
    std::string DescribeVerbose() const;
    std::string DescribeOperation() const;
};

/**
 * @brief device query that holds values.
 *  Used to write values to devices.
 */
struct TIRDeviceValueQuery final: TIRDeviceQuery
{
    friend class TIRDeviceQueryFactory;

    const TPSet<PMemoryBlock> MemoryBlocks;
    std::vector<TIRDeviceValueContext> ValueContexts;

    explicit TIRDeviceValueQuery(TAssociatedMemoryBlockSet &&, EQueryOperation = EQueryOperation::Write);
    explicit TIRDeviceValueQuery(TPSet<PMemoryBlock> &&, EQueryOperation = EQueryOperation::Write);

    void AddValueContext(const TIRDeviceValueContext &);

    TIRDeviceMemoryView GetValues(void * mem, size_t size) const
    {
        return GetValuesImpl(mem, size);
    }

    TIRDeviceMemoryView GetValues(std::vector<uint8_t> & values) const
    {
        values.resize(GetSize());

        return GetValuesImpl(values.data(), values.size());
    }

    /**
     * @brief Accept written values to device, update cache and set status to Ok
     */
    void FinalizeWrite() const;

private:
    TIRDeviceMemoryView GetValuesImpl(void * mem, size_t size) const;
};

/**
 * @brief set of device queries.
 *
 * @note when creating set for each poll interval,
 *  eases dynamic query subdivision on errors,
 *  by allowing to modify query set instead of polling plan
 */
struct TIRDeviceQuerySet
{
    friend class TIRDeviceQuerySetHandler;

    TQueries Queries;

    TIRDeviceQuerySet(const std::vector<PVirtualRegister> &, EQueryOperation);

    std::string Describe() const;
    PSerialDevice GetDevice() const;
};
