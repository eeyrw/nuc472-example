
# STD Defines
DDEFS += 

# source director
PERIPH_STD_LIB     = $(LIB_DIR)/StdDriver
DEVICE_LIB  = $(LIB_DIR)/Device/Nuvoton/NUC472_442
PERIPH_STD_INC_DIR     = $(PERIPH_STD_LIB)/inc
PERIPH_STD_SRC_DIR     = $(PERIPH_STD_LIB)/src
CMSIS_INC_DIR     = $(LIB_DIR)/CMSIS/Include
# startup
SRC  += $(DEVICE_LIB)/Source/GCC/startup.c


# use libraries, please add or remove when you use or remove it.
SRC  += $(PERIPH_STD_SRC_DIR)/clk.c
SRC  += $(PERIPH_STD_SRC_DIR)/gpio.c
SRC  += $(PERIPH_STD_SRC_DIR)/fmc.c
SRC  += $(PERIPH_STD_SRC_DIR)/sys.c

# include directories
INCLUDE_DIRS += $(STM32F1_CORE_DIR)
INCLUDE_DIRS += $(DEVICE_LIB)/Include
INCLUDE_DIRS += $(PERIPH_STD_INC_DIR)
INCLUDE_DIRS += $(PERIPH_STD_SRC_DIR)
INCLUDE_DIRS += $(PERIPH_STD_LIB)
INCLUDE_DIRS += $(CMSIS_INC_DIR)

