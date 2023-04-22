#pragma once

#include <tuple>
#include <stdint.h>
#include <type_traits>

/*
* How to use:
*                                                                 unsafe step (unsigned typed integer)
*                                 type pointed at  1st pointer	  (nullptr check before adding offset)
*                                        vvv        vvvvvvvvvvv     vvv
* auto pointerChain = PointerChain::make<int, true>(pointerBase, 8, 40u, dynamicOffset, 24);
*                                             ^^^^                       ^^^^^^^^^^^^^
*   standard nullptr check               check for nullptr               an integer variable
*   (will check every important* step)   every pointer step	         (referenced)
*     v
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
	// Implementation helpers for constructing and traversing pointer chains.
	namespace Impl {
		// Get the depth of a pointer, i.e. how many times it needs to be dereferenced to get to the original object.
		template <typename T> struct pointer_depth {
			static constexpr std::size_t value = 0;
		};

		template<typename T> struct pointer_depth<T*> {
			static constexpr std::size_t value = 1 + pointer_depth<T>::value;
		};

		template <typename T> constexpr std::size_t pointer_depth_v = pointer_depth<T>::value;

		// Create a default initialized tuple of size N with types T https://stackoverflow.com/a/37094024
		template<std::size_t, class T> using T_ = T;

		template<typename T, std::size_t... Is> constexpr auto generate_tuple(std::index_sequence<Is...>) noexcept
		{
			return std::tuple<T_<Is, T>...>{};
		}

		template<typename T, std::size_t N> constexpr auto generate_tuple() noexcept
		{
			return generate_tuple<T>(std::make_index_sequence<N>{});
		}

		// Create a tuple from another tuple and an index sequence.
		template <typename... Ts, std::size_t... I> constexpr auto subtuple(const std::tuple<Ts...>& t, std::index_sequence<I...>) noexcept
		{
			return std::make_tuple(std::get<I>(t)...);
		}

		// Check if a type is an immediate value or a variable, only works with forwarding references.
		template <typename T> struct is_immediate_value
		{
			static constexpr bool value = std::is_integral_v<std::remove_reference_t<T>>;
		};

		template <typename T> constexpr bool is_immediate_value_v = is_immediate_value<T>::value;
	}

	// Base PointerChain class
	template <typename PointerType_, bool null_safe_ = false, typename Tb_ = uintptr_t, typename... Offsets_> class PtrChainBase {
		static_assert(std::conjunction_v<std::is_integral<std::remove_reference_t<Offsets_>>...>, "Offset type is not explicitly an integer type.");

	public:
		// No default constructor, only PointerChain::make can be used.
		PtrChainBase() = delete;
		virtual ~PtrChainBase() {}

		PtrChainBase(const PtrChainBase& other) : base(other.base), offsets(other.offsets) {}
		PtrChainBase(PtrChainBase&& other) : base(std::move(other.base)), offsets(std::move(other.offsets)) {}

		// Traverse the first N offsets of the pointer chain (default = all) and return a pointer from the last offset traversed.
		template <std::size_t N = std::tuple_size<std::tuple<Offsets_...>>::value> constexpr auto get() noexcept
		{
			if constexpr (N < std::tuple_size<std::tuple<Offsets_...>>::value) {
				return this->apply_(Impl::subtuple(this->offsets, std::make_index_sequence<N>()));
			}
			else {
				return reinterpret_cast<PointerType_*>(this->apply_(this->offsets));
			}
		}

		// Dereference the chain and check for nullptr.
		constexpr operator bool() noexcept
		{
			return !!this->get();
		}

		// Comparison operator for nullptr.
		constexpr bool operator != (std::nullptr_t null) noexcept
		{
			return this->get() != null;
		}

		// Comparison operator for nullptr.
		constexpr bool operator == (std::nullptr_t null) noexcept
		{
			return this->get() == null;
		}

		// Dereference the chain.
		constexpr PointerType_& operator *() noexcept
		{
			return *this->get();
		}

		// Dereference the chain with an arrow operator.
		constexpr PointerType_* operator ->() noexcept
		{
			return this->get();
		}

		// Returns the current value at an offset.
		template<std::size_t I> constexpr auto getOffset() const noexcept
		{
			return std::get<I>(offsets);
		}

		// Returns the total number of offsets in the chain.
		constexpr std::size_t getNumOffsets() const noexcept
		{
			return std::tuple_size<std::tuple<Offsets_...>>::value;
		}

	private:
		const Tb_ base;
		const std::tuple<Offsets_...> offsets;
		// Private constructor - use PointerChain::make instead.
		PtrChainBase(Tb_ base, Offsets_... offsets) : base(base), offsets(std::forward_as_tuple<Offsets_&...>(offsets...)) {}

		template <typename PointerType, bool null_safe, typename Tb, typename... Offsets> friend inline constexpr auto make(Tb base, Offsets&&... offsets) noexcept;

		// Applies a tuple to a traversal function. If null_safe was set at instantiation, nullptr checks will be added every step of the chain.
		template <typename T = void, typename... Ts> constexpr T* apply_(const std::tuple<Ts...>& t) noexcept
		{
			if constexpr (null_safe_) {
				return std::apply([this](auto &&... args) constexpr -> T* { return reinterpret_cast<T*>(this->traverse(this->base, static_cast<const unsigned int&>(args)...)); }, t);
			}
			else {
				return std::apply([this](auto &&... args) constexpr -> T* { return reinterpret_cast<T*>(this->traverse(this->base, args...)); }, t);
			}
		}

		// Chain traversal functions. If an offset is of an unsigned type, perform a nullptr check before adding it.
		template <typename Tb, typename To> constexpr void* traverse(Tb base, const To& offset0) noexcept
		{
			if constexpr (std::is_same_v<To, unsigned int>) {
				if (!base) return nullptr;
			}
			return reinterpret_cast<void*>(reinterpret_cast<unsigned char*>(base) + static_cast<int>(offset0));
		}

		template <typename Tb, typename To, typename... Args> constexpr void* traverse(Tb base, const To& offset0, const Args... offsets) noexcept
		{
			if constexpr (std::is_same_v<To, unsigned int>) {
				if (!base) return nullptr;
			}
			return traverse(*reinterpret_cast<uintptr_t*>(static_cast<uintptr_t>(base) + static_cast<int>(offset0)), offsets...);
		}
	};

	// Only way to construct a pointer chain. PointerType is the type pointed to by the chain, null_safe dictates whether EVERY offset should be treated as unsafe (unsigned).
	// The base can be of any type, references and pointers are dereferenced when traversing the pointer!
	// The offsets can either be immediate integral values or variables of types that can be implicitly converted to such.
	// Keep in mind any variable passed to PointerChain::make will be stored as a reference to that variable.
	template <typename PointerType = unsigned char, bool null_safe = false, typename Tb, typename... Offsets> inline constexpr auto make(Tb base, Offsets&&... offsets) noexcept
	{
		auto base_ = reinterpret_cast<const uintptr_t&>(base);
		constexpr int64_t pointerDepth = Impl::pointer_depth_v<Tb> + std::is_reference_v<Tb> - 1;

		if constexpr (pointerDepth <= 0) {
			return PtrChainBase<PointerType, null_safe, decltype(base_), std::conditional_t<Impl::is_immediate_value_v<Offsets>, std::decay_t<Offsets>, Offsets&>...>(base_, offsets...);
		}
		else {
			constexpr auto addOffsets = Impl::generate_tuple<int, pointerDepth>();
			return std::apply([&](auto &&... args)->auto { return PtrChainBase<PointerType, null_safe, decltype(base_), std::conditional_t<Impl::is_immediate_value_v<decltype(args)>, std::decay_t<decltype(args)>, decltype(args)&>...>(base_, args...); }, std::tuple_cat(addOffsets, std::forward_as_tuple(offsets...)));
		}
	}
}
