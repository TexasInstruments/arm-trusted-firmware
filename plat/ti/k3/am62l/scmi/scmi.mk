CONFIG_CLK_PLL_16FFT_FRACF_CALIBRATION := 1


PLAT_INCLUDES		+=	\
				-I${PLAT_PATH}/am62l/scmi			\
				-I${PLAT_PATH}/am62l/scmi/drivers/include/			\
				-I${PLAT_PATH}/am62l/scmi/drivers/include/soc/			\
				-I${PLAT_PATH}/am62l/scmi/drivers/soc/am62lx/include/			\
				-I${PLAT_PATH}/am62l/scmi/drivers/soc/am62lx/include/soc/am62lx/			\

BL31_SOURCES		+=	\
				${PLAT_PATH}/am62l/scmi/scmi.c	\
				${PLAT_PATH}/am62l/scmi/scmi_clock.c	\
				${PLAT_PATH}/am62l/scmi/scmi_pd.c	\
