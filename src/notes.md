# Allocator

```rust
use embedded_alloc::Heap;

#[global_allocator]
static HEAP: Heap = Heap::empty();

#[macro_use]
extern crate alloc;

//main
// Initialize the allocator BEFORE you use it
{
    use core::mem::MaybeUninit;
    const HEAP_SIZE: usize = 1024;
    static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];
    unsafe { HEAP.init(HEAP_MEM.as_ptr() as usize, HEAP_SIZE) }
}

```

# Embedded Graphics

```rust
use embedded_graphics::{
    mono_font::{ascii::FONT_8X13, MonoTextStyleBuilder},
    pixelcolor::BinaryColor,
    prelude::*,
    text::{Baseline, Text},
};


    let text_style = MonoTextStyleBuilder::new()
        .font(&FONT_8X13)
        .text_color(BinaryColor::On)
        .build();

    Text::with_baseline("Hello world!", Point::zero(), text_style, Baseline::Top)
        .draw(&mut display)
        .unwrap();
    Text::with_baseline("Hello rust!", Point::new(0, 16), text_style, Baseline::Top)
        .draw(&mut display)
        .unwrap();
    display.flush().unwrap();

```