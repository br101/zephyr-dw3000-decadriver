SECTION_PROLOGUE(.dw_drivers,,)
{
. = ALIGN(4);
dw_drivers_load_start = .;
dw_drivers_start = .;
__dw_drivers_start = dw_drivers_start;
KEEP(*(.dw_drivers .dw_drivers.))
. = ALIGN(4);
} GROUP_LINK_IN(ROMABLE_REGION)

dw_drivers_end = dw_drivers_start + SIZEOF(.dw_drivers);
dw_drivers_size = SIZEOF(.dw_drivers);
__dw_drivers_end = dw_drivers_end;
dw_drivers_load_end = dw_drivers_end;
