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
		// A helper make function, use PointerChain::make instead.
		template <typename PointerType, bool null_safe, std::size_t extra_offset_count = 0, bool decay_immediate = true, typename Tb, typename... Ts> inline constexpr auto make(const Tb& base, const std::tuple<Ts...>& t) noexcept;

		template <typename T> struct pointer_depth {
			static constexpr std::size_t value = 0;
		};

		template<typename T> struct pointer_depth<T*> {
			static constexpr std::size_t value = 1 + pointer_depth<T>::value;
		};

		// Get the depth of a pointer, i.e. how many times it needs to be dereferenced to get to the original object.
		template <typename T> constexpr std::size_t pointer_depth_v = pointer_depth<T>::value;

		// Get the size of a parameter pack, i.e. the number of arguments passed.
		template <typename... Ts> constexpr std::size_t pack_size_v = sizeof...(Ts);

		// Check if two objects are from the same template class, regardless of template arguments.
		template <typename... Ts> struct is_same_template_class {
			static constexpr bool value = false;
		};

		// Check if two objects are from the same template class, regardless of template arguments.
		template <template <typename...> class T, typename... Ts, typename... Us> struct is_same_template_class<T<Ts...>, T<Us...>> {
			static constexpr bool value = true;
		};

		template<std::size_t, class T> using T_ = T;

		template<typename T, std::size_t... Is> constexpr auto generate_tuple(std::index_sequence<Is...>) noexcept
		{
			return std::tuple<T_<Is, T>...>{};
		}

		// Create a default initialized tuple of size N with types T https://stackoverflow.com/a/37094024
		template<typename T, std::size_t N> constexpr auto generate_tuple() noexcept
		{
			return generate_tuple<T>(std::make_index_sequence<N>{});
		}

		template <std::size_t Start, std::size_t... I, typename... Ts> constexpr auto subtuple(const std::tuple<Ts...>& t, std::index_sequence<I...>) noexcept
		{
			return std::make_tuple(std::get<Start + I>(t)...);
		}

		// Create a subtuple from elements from indexes Start to End.
		template <std::size_t Start = 0, std::size_t End, typename... Ts> constexpr auto subtuple(const std::tuple<Ts...>& t) noexcept
		{
			return subtuple<Start>(t, std::make_index_sequence<End - Start + 1>{});
		}

		template <typename T> struct is_immediate_value
		{
			static constexpr bool value = std::is_integral_v<std::remove_reference_t<T>>;
		};

		// Check if a type is an immediate value or a variable, only works with forwarding references.
		template <typename T> constexpr bool is_immediate_value_v = is_immediate_value<T>::value;

		// A wrapper for storing offsets of reference offsets.
		template <typename T_ = int&> class ref_offset_wrapper {
		public:
			ref_offset_wrapper() = delete;
			T_ ref_offset;
			int offset;

			using wrapped_type = std::conditional_t<is_same_template_class<ref_offset_wrapper<>, T_>::value, typename T_::wrapped_type, T_>;

			// Will recursively add up all of the offsets in wrappers and wrapped wrappers.
			constexpr operator int() const noexcept
			{
				return static_cast<int>(ref_offset) + offset;
			}

		private:
			ref_offset_wrapper(T_ ref_offset, int offset) : ref_offset(ref_offset), offset(offset) {
				static_assert(std::is_same_v<decltype(ref_offset), const int&>);
			}
			template <typename T> friend constexpr auto make_ref_offset_wrapper(T& ref_offset, int offset);
		};

		// Makes a ref_offset_wrapper, storing a reference offset as such and stripping references when wrapping another ref_offset_wrapper.
		template <typename T> constexpr auto make_ref_offset_wrapper(T& ref_offset, int offset)
		{
			if constexpr (Impl::is_same_template_class<Impl::ref_offset_wrapper<>, T>::value) {
				return ref_offset_wrapper([](T strip_ref) constexpr -> T { return strip_ref; }(ref_offset), offset);
			}
			else {
				static_assert(std::is_same_v<decltype(ref_offset), const int&>);
				return ref_offset_wrapper<T&>(ref_offset, offset);
			}
		}

		// Unwraps the underlying type of a ref_offset_wrapper.
		template <typename T> struct unwrap_type {
			using type = typename std::decay_t<T>;
		};

		// Unwraps the underlying type of a ref_offset_wrapper.
		template <typename T> struct unwrap_type<ref_offset_wrapper<T>> {
			using type = typename unwrap_type<T>::type;
		};
	}

	// Base PointerChain class
	template <typename PointerType_, bool null_safe_ = false, std::size_t extra_offset_count_ = 0, typename Tb_ = uintptr_t, typename... Offsets_> class PtrChainBase {
		static_assert(std::conjunction_v<std::is_integral<std::decay_t<Offsets_>>...>
			|| std::disjunction_v<Impl::is_same_template_class<Impl::ref_offset_wrapper<>, std::decay_t<Offsets_>>...>,
			"Offset type must explicitly be an integer type.");

	public:
		// No default constructor, only PointerChain::make can be used.
		PtrChainBase() = delete;
		virtual ~PtrChainBase() {}

		PtrChainBase(const PtrChainBase& other) : base(other.base), offsets(other.offsets) {}
		PtrChainBase(PtrChainBase&& other) : base(std::move(other.base)), offsets(std::move(other.offsets)) {}

		// Create a chain with a new pointed to type.
		template <typename TOther> constexpr auto to() const
		{
			return Impl::make<TOther, null_safe_, extra_offset_count_>(this->base, this->offsets);
		}

		// Traverse offsets up until and including N (default = all) and return a pointer from the last offset traversed.
		// At least one offset needs to be traversed or the program is ill-formed.
		template <std::size_t N = Impl::pack_size_v<Offsets_...> - extra_offset_count_ - 1> constexpr auto get() noexcept
		{
			static_assert(N < Impl::pack_size_v<Offsets_...> - extra_offset_count_, "N cannot be greater than or equal to the total number of offsets");
			if constexpr (N < Impl::pack_size_v<Offsets_...> - extra_offset_count_ - 1) {
				return this->apply_(Impl::subtuple<0, extra_offset_count_ + N>(this->offsets));
			}
			else {
				return reinterpret_cast<PointerType_*>(this->apply_(this->offsets));
			}
		}

		// Create a new chain with an offset added to the last offset of the original.
		template <typename T> constexpr auto operator + (T offset) const noexcept
		{
			static_assert(std::is_integral_v<T>, "Offset type must explicitly be an integer type.");
			constexpr std::size_t offsetNum = Impl::pack_size_v<Offsets_...>;
			constexpr std::size_t lastIndex = offsetNum - 1;
			if constexpr (std::is_reference_v<std::tuple_element_t<lastIndex, std::tuple<Offsets_...>>>
				|| std::is_base_of_v<Impl::ref_offset_wrapper<>, std::tuple_element_t<lastIndex, std::tuple<Offsets_...>>>) {
				auto ref_wrapper = std::make_tuple(Impl::make_ref_offset_wrapper(std::get<lastIndex>(this->offsets), offset));
				if constexpr (offsetNum > 1) {
					auto t = std::tuple_cat(Impl::subtuple<0, lastIndex - 1>(this->offsets), ref_wrapper);
					return Impl::make<PointerType_, null_safe_, extra_offset_count_>(this->base, t);
				}
				else {
					return Impl::make<PointerType_, null_safe_, extra_offset_count_>(this->base, ref_wrapper);
				}
			}
			else {
				auto t = std::tuple_cat(Impl::subtuple<0, lastIndex - 1>(this->offsets), std::make_tuple(std::get<lastIndex>(this->offsets) + offset));
				return Impl::make<PointerType_, null_safe_, extra_offset_count_>(this->base, t);
			}
		}

		// Create a new chain with an offset subracted from the last offset of the original.
		template <typename T> constexpr auto operator - (T offset) const noexcept
		{
			static_assert(std::is_integral_v<T>, "Offset type must explicitly be an integer type.");
			constexpr std::size_t offsetNum = Impl::pack_size_v<Offsets_...>;
			constexpr std::size_t lastIndex = offsetNum - 1;
			if constexpr (std::is_reference_v<std::tuple_element_t<lastIndex, std::tuple<Offsets_...>>>
				|| std::is_base_of_v<Impl::ref_offset_wrapper<>, std::tuple_element_t<lastIndex, std::tuple<Offsets_...>>>) {
				auto ref_wrapper = std::make_tuple(Impl::make_ref_offset_wrapper(std::get<lastIndex>(this->offsets), -offset));
				if constexpr (offsetNum > 1) {
					auto t = std::tuple_cat(Impl::subtuple<0, lastIndex - 1>(this->offsets), ref_wrapper);
					return Impl::make<PointerType_, null_safe_, extra_offset_count_>(this->base, t);
				}
				else {
					return Impl::make<PointerType_, null_safe_, extra_offset_count_>(this->base, ref_wrapper);
				}
			}
			else {
				auto t = std::tuple_cat(Impl::subtuple<0, lastIndex - 1>(this->offsets), std::make_tuple(std::get<lastIndex>(this->offsets) - offset));
				return Impl::make<PointerType_, null_safe_, extra_offset_count_>(this->base, t);
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

		// Dereference the pointer chain.
		constexpr PointerType_& operator *() noexcept
		{
			return *this->get();
		}

		// Dereference the chain with an arrow operator.
		constexpr PointerType_* operator ->() noexcept
		{
			return this->get();
		}

		// Dereference function.
		constexpr PointerType_& dereference() noexcept
		{
			return *this->get();
		}

		// Dereference function with a fallback value returned when dereferencing returns nullptr.
		constexpr PointerType_& dereference(PointerType_&& fallback)
		{
			PointerType_* ptr = this->get();
			return !!ptr ? *ptr : fallback;
		}

		// Returns the current value at an offset, defaults to the last one in the chain.
		template<std::size_t I = Impl::pack_size_v<Offsets_...> + extra_offset_count_ - 1> constexpr auto getOffset() const noexcept
		{
			static_assert(I < Impl::pack_size_v<Offsets_...> - extra_offset_count_, "Offset index out of bounds.");
			return std::get<I + extra_offset_count_>(offsets);
		}

		// Returns the total number of offsets in the chain.
		constexpr std::size_t getNumOffsets() const noexcept
		{
			return Impl::pack_size_v<Offsets_...> - extra_offset_count_;
		}

	private:
		const Tb_ base;
		const std::tuple<Offsets_...> offsets;
		// Private constructor - use PointerChain::make instead.
		PtrChainBase(Tb_ base, Offsets_... offsets) : base(base), offsets(std::forward_as_tuple<Offsets_&...>(offsets...)) {}

		template <typename PointerType, bool null_safe, std::size_t extra_offset_count, bool decay_immediate, typename Tb, typename... Ts> friend inline constexpr auto Impl::make(const Tb& base, const std::tuple<Ts...>& t) noexcept;

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
			if constexpr (std::is_same_v<typename Impl::unwrap_type<To>::type, unsigned int>) {
				if (!base) return nullptr;
			}
			return reinterpret_cast<void*>(reinterpret_cast<unsigned char*>(base) + static_cast<int>(offset0));
		}

		template <typename Tb, typename To, typename... Args> constexpr void* traverse(Tb base, const To& offset0, const Args... offsets) noexcept
		{
			if constexpr (std::is_same_v<typename Impl::unwrap_type<To>::type, unsigned int>) {
				if (!base) return nullptr;
			}
			return traverse(*reinterpret_cast<uintptr_t*>(static_cast<uintptr_t>(base) + static_cast<int>(offset0)), offsets...);
		}
	};

	template <typename PointerType, bool null_safe, std::size_t extra_offset_count, bool decay_immediate, typename Tb, typename... Ts> inline constexpr auto Impl::make(const Tb& base, const std::tuple<Ts...>& t) noexcept
	{
		if constexpr (decay_immediate) {
			return std::apply([&](auto &&... args) constexpr -> auto { return PtrChainBase<PointerType, null_safe, extra_offset_count, Tb, std::conditional_t<Impl::is_immediate_value_v<decltype(args)>, std::decay_t<decltype(args)>, decltype(args)&>...>(base, args...); }, t);
		}
		else {
			return std::apply([&](auto &&... args) constexpr -> auto { return PtrChainBase<PointerType, null_safe, extra_offset_count, Tb, decltype(args)...>(base, args...); }, t);
		}
	}

	// Only way to construct a pointer chain. PointerType is the type pointed to by the chain, null_safe dictates whether EVERY offset should be treated as unsafe (unsigned).
	// The base can be of any type, references and pointers are dereferenced when traversing the pointer!
	// The offsets can either be immediate integral values or variables of types that can be implicitly converted to such.
	// Keep in mind any variable passed to PointerChain::make will be stored as a reference to that variable.
	// Only way to construct a pointer chain. PointerType is the type pointed to by the chain, null_safe dictates whether EVERY offset should be treated as unsafe (unsigned).
	template <typename PointerType = unsigned char, bool null_safe = false, typename Tb, typename... Offsets> inline constexpr auto make(Tb& base, Offsets&&... offsets) noexcept
	{
		auto base_ = reinterpret_cast<const uintptr_t&>(base);
		constexpr int64_t pointerDepth = Impl::pointer_depth_v<Tb> + std::is_reference_v<Tb> - 1;

		if constexpr (pointerDepth <= 0) {
			auto t = std::forward_as_tuple(offsets...);
			return Impl::make<PointerType, null_safe>(base_, t);
		}
		else {
			auto t = std::tuple_cat(Impl::generate_tuple<std::conditional_t<null_safe, unsigned int, int>, pointerDepth>(), std::forward_as_tuple(offsets...));
			return Impl::make<PointerType, null_safe, pointerDepth>(base_, t);
		}
	}
}
