# PointerChain
Cheat Engine-like pointer chains for C++17, simple and efficient to use

# Example:
https://godbolt.org/z/dh19xhMef
```cpp
int main()
{
    // A simple pointer to the player's health as a 32-bit integer, 
    // note the 3rd offset marked unsigned, a nullptr check will be performed
    // before adding it whenever the pointer is traversed
    auto playerHP = PointerChain::make<int>(WorldChrMan, 0x10EF8, 0, 0x190u, 0, 0x138); // health

    // A "safe" version of the same pointer that performs nullptr checks every step
    auto playerHPSafe = PointerChain::make<int, true>(WorldChrMan, 0x10EF8, 0, 0x190, 0, 0x138);

    // You can add and substract from the last offset of the pointer chain, 
    // creating new pointer chain instances
    auto playerSP = playerHP + 0x20; // stamina
    auto playerMP = playerSP - 0x10; // focus

    // Dereference a pointer and check for nullptr:
    if (!playerHP) /* playerHP != nullptr also works */ {
        return 0;
    }

    // Dereference a pointer chain to add 10 focus to the player 
    *playerMP += 10; 
    
    // It is possible to create pointers with dynamic offsets.
    // These offsets will be stored as references, so the pointer can be adjusted dynamically
    auto dynamicExample = PointerChain::make<bool, true>(WorldChrMan, 0x10, dynamicOffset1, 0x0, dynamicOffset2);

    // You can likewise add integral variables to the last offset of the pointer chain.
    // They will also be stored as references and may be adjusted.
    // Offsets and variables can be substracted from the last offset of the pointer chain.
    int addOffset = 0x10;
    auto dynamicExample2 = dynamicExample + addOffset - negativeOffset + 0x8;

    // A pointer chain can be dereferenced with the PointerChain::dereference method.
    // You may provide a fallback value that will be returned if
    // one of the steps of the chain with a nullptr check resolves to nullptr.
    if (dynamicExample2.dereference(true)) {
        return 0;
    }

    // You may traverse the first N offsets of a chain with PointerChain::get<N>.
    // Note that the return value is a void pointer unless the chain is fully traversed.
    void* playerModules = playerHPSafe.get<4>();

    // The pointed to object type can be cast with PointerChain::to<T>;
    auto currentPlayerAnimation = PointerChain::make<int>(playerModules, 0x20u);
    auto currentPlayerAnimationTime = currentPlayerAnimation.to<float>() + 0x4;
    
    // A pointer chain can be implicitly cast to a boolean,
    // representing a nullptr check.
    if (currentPlayerAnimationTime && *currentPlayerAnimationTime > 1.5f) {
        return 1;
    }

    // Most of the calculations above are performed at compile time.
    // Only the minimal needed steps will be taken at runtime, ensuring efficiency.
    return 0;
}
```
