#pragma once

#include <tuple>
#include <stdint.h>
#include <type_traits>

/*
* How to use:
*																  unsafe step (unsigned typed integer)
*								   type pointed at  1st pointer	  (nullptr check before adding offset)
*										 vvv		vvvvvvvvvvv		vvv
* auto pointerChain = PointerChain::make<int, true>(pointerBase, 8, 40u, dynamicOffset, 24);
*											  ^^^^						 ^^^^^^^^^^^^^
*	standard nullptr check      			  check for nullptr			 an integer variable
*	(will check every important* step)  	  every pointer step	     (referenced)
*	  v
* if (!pointerChain) {...} // *every unsigned step, the final offset and/or all of them if nullptr_safe is set (it is in the example above)
* 
* int* pVal = pointerChain.get(); // PointerChain::get() for a pointer from the last offset
* int val = *pointerChain; // dereference operator to fully dereference chain (be careful with nullptr!)
*/

/// <summary>
/// A tuple wrapper focused on convenient compile time pointer management. 
/// Represents a continous chain of pointers and offsets, so that the n-th pointer offset by the n-th offset points to the next pointer in the chain.
/// A chain with an invalid base or consisting of just a base is undefined.
/// </summary>
namespace PointerChain {
	template <typename PointerType_, bool null_safe_ = false, typename... Offsets_> class PtrChainBase {
		static_assert(std::conjunction_v<std::is_integral<std::remove_reference_t<Offsets_>>...>, "Offset type is not explicitly an integer type.");

	public:
		PtrChainBase() = delete;
		virtual ~PtrChainBase() {}

		// Traverse the first N offsets of the pointer chain (default = all) and return a pointer from the last offset traversed.
		template <std::size_t N = std::tuple_size<std::tuple<Offsets_...>>::value> constexpr auto get()
		{
			if constexpr (N < std::tuple_size<std::tuple<Offsets_...>>::value) {
				return this->apply_(subtuple_(this->offsets, std::make_index_sequence<N>()));
			}
			else {
				return reinterpret_cast<PointerType_*>(this->apply_(this->offsets));
			}
		}

		// Dereference the chain and check for nullptr.
		constexpr bool operator !()
		{
			return !this->get<std::tuple_size<std::tuple<Offsets_...>>::value>();
		}

		// Dereference the chain.
		constexpr PointerType_& operator *()
		{
			return *this->get();
		}

		// Returns the current value at an offset.
		template<std::size_t I> constexpr auto getOffset() const
		{
			return std::get<I>(offsets);
		}

		// Returns the total number of offsets in the chain.
		constexpr std::size_t getNumOffsets() const
		{
			return std::tuple_size<std::tuple<Offsets_...>>::value;
		}

	private:
		void*& base;
		const std::tuple<Offsets_&...> offsets;
		PtrChainBase(void*& base, const Offsets_&... offsets) : base(base), offsets(std::forward_as_tuple<const Offsets_&...>(offsets...)) {}

		template <typename PointerType, bool null_safe, typename T, typename... Offsets> friend constexpr auto make(T*& base, const Offsets&... offsets);

		// Applies a tuple to a traversal function. If null_safe was set at instantiation, nullptr checks will be added every step of the chain.
		template <typename T = void, typename... Ts> constexpr T* apply_(const std::tuple<Ts...>& t)
		{
			if constexpr (null_safe_) {
				return std::apply([this](auto &&... args) -> T* { return reinterpret_cast<T*>(this->traverse(this->base, static_cast<const unsigned int&>(args)...)); }, t);
			}
			else {
				return std::apply([this](auto &&... args) -> T* { return reinterpret_cast<T*>(this->traverse(this->base, args...)); }, t);
			}
		}

		template <typename... Ts, std::size_t... I> constexpr auto subtuple_(const std::tuple<Ts...>& t, std::index_sequence<I...>)
		{
			return std::make_tuple(std::get<I>(t)...);
		}

		// Chain traversal functions. If an offset is of an unsigned type, perform a nullptr check before adding it.

		template <typename T> constexpr void* traverse(void*& base, const T& offset0)
		{
			if constexpr (std::is_same_v<T, unsigned int>) {
				if (!base) return nullptr;
			}
			return reinterpret_cast<void*>(reinterpret_cast<unsigned char*>(base) + static_cast<int>(offset0));
		}

		template <typename T> constexpr void* traverse(void**& base, const T& offset0)
		{
			if (!base) return nullptr;
			if constexpr (std::is_same_v<T, unsigned int>) {
				if (!*base) return nullptr;
			}
			return reinterpret_cast<void*>(reinterpret_cast<unsigned char*>(*base) + static_cast<int>(offset0));
		}

		template <typename T, typename... Args> constexpr void* traverse(void*& base, const T& offset0, const Args... offsets)
		{
			if constexpr (std::is_same_v<T, unsigned int>) {
				if (!base) return nullptr;
			}
			return traverse(reinterpret_cast<void*&>(*reinterpret_cast<uintptr_t*>(reinterpret_cast<uintptr_t>(base) + static_cast<int>(offset0))), offsets...);
		}

		template <typename T, typename... Args> constexpr void* traverse(void**& base, const T& offset0, const Args... offsets)
		{
			if (!base) return nullptr;
			if constexpr (std::is_same_v<T, unsigned int>) {
				if (!*base) return nullptr;
			}
			return traverse(reinterpret_cast<void*&>(*reinterpret_cast<uintptr_t*>(reinterpret_cast<unsigned char*>(*base) + static_cast<int>(offset0))), offsets...);
		}
	};

	// Only way to construct a pointer chain. PointerType is the type pointed to by the chain, null_safe dictates whether EVERY offset should be treated as unsafe (unsigned).
	template <typename PointerType, bool null_safe = false, typename T, typename... Offsets> static constexpr auto make(T*& base, const Offsets&... offsets)
	{
		return PtrChainBase<PointerType, null_safe, const Offsets&...>(base, offsets...);
	}
}