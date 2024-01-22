/*
	Default linker script for MD407 (STM32F407)
	All code and data goes to RAM.
*/
/* Memory Spaces Definitions */
MEMORY
{
  FLASH (rx) : ORIGIN = 0x08000000, LENGTH = 1024K
	RAM (xrw) : ORIGIN = 0x20000000, LENGTH = 112K
}

_stack_start = ORIGIN(RAM) + LENGTH(RAM);
