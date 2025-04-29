//Perform any necessary initialization for the touchscreen and associated ADC pins.
void touch_init(void); 

// Configure pins and associated ADC to sample either the x- or y-dimension
void touch_select_dim(uint8_t dimension);

// Acquire a position sample from the touchscreen on the dimension selected with the previous 
// touch_select_dim(...) command
uint16_t touch_read(void);
