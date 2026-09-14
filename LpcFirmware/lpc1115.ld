ENTRY(Reset_Handler)

MEMORY
{
	FLASH (rx)  : ORIGIN = 0x00000000, LENGTH = 64K
	RAM (rwx)   : ORIGIN = 0x10000000, LENGTH = 8K
}

_estack = ORIGIN(RAM) + LENGTH(RAM);

/* The LPC11xx boot ROM requires the first eight vector words to sum to zero.
 * Thumb function pointers stored in the vector table have bit 0 set, hence +3
 * for Reset_Handler and the two Default_Handler entries in words 1..3. */
__vector_checksum = 0 - (_estack + Reset_Handler + (2 * Default_Handler) + 3);

SECTIONS
{
	.isr_vector :
	{
		KEEP(*(.isr_vector))
	} > FLASH

	.crp 0x000002FC :
	{
		KEEP(*(.crp))
	} > FLASH

	ASSERT(SIZEOF(.crp) == 4, "LPC1115 CRP word must be exactly 4 bytes")

	.text :
	{
		*(.text*)
		*(.rodata*)
		. = ALIGN(4);
	} > FLASH

	.data :
	{
		. = ALIGN(4);
		_sdata = .;
		*(.data*)
		. = ALIGN(4);
		_edata = .;
	} > RAM AT > FLASH
	_sidata = LOADADDR(.data);

	.bss (NOLOAD) :
	{
		. = ALIGN(4);
		_sbss = .;
		*(.bss*)
		*(COMMON)
		. = ALIGN(4);
		_ebss = .;
	} > RAM

	/DISCARD/ :
	{
		*(.comment*)
		*(.note*)
	}
}

ASSERT(_ebss <= _estack, "LPC1115 RAM overflow")
