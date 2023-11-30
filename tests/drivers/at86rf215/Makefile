INCLUDES += -I$(APPDIR)
BOARD ?= TC3_Mesh

include ../Makefile.drivers_common

USEMODULE += main_app_cmx638
INCLUDES += -I$(CURDIR)/CMX638
DIRS += $(CURDIR)/CMX638

USEMODULE += test_utils_netdev_ieee802154_minimal

# the radio driver to test
USEMODULE += at86rf215
USEMODULE += at86rf215_batmon
USEMODULE += netdev_ieee802154_rx_timestamp
USEMODULE += netdev_ieee802154_mr_ofdm

CFLAGS += -DEVENT_THREAD_STACKSIZE_DEFAULT=1024
CFLAGS += -Wno-unused-variable
CFLAGS += -Wno-unused-but-set-variable
CFLAGS += -Wno-unused-parameter
CFLAGS += -Wno-unused-function
CFLAGS += -DCONFIG_IEEE802154_DEFAULT_PHY_MR_OFDM=1
CFLAGS += -DCONFIG_AT86RF215_DEFAULT_MR_OFDM=1
include $(RIOTBASE)/Makefile.include
