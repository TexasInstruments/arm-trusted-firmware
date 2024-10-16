int32_t scmi_handler_clock_prepare(uint32_t dev_id, uint32_t clk_id);
int32_t scmi_handler_clock_unprepare(uint32_t dev_id, uint32_t clk_id);
int32_t scmi_handler_clock_set_rate(uint32_t dev_id, uint32_t clk_id, uint64_t target_freq);
uint64_t scmi_handler_clock_get_rate(uint32_t dev_id, uint32_t clk_id);
