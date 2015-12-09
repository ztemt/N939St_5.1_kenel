#
# Makefile for nfc devices
#

obj-$(CONFIG_NFC_PN544)		+= pn544/
obj-$(CONFIG_NFC_MICROREAD)	+= microread/
obj-$(CONFIG_NFC_PN533)		+= pn533.o
obj-$(CONFIG_NFC_WILINK)	+= nfcwilink.o
obj-$(CONFIG_NFC_MEI_PHY)	+= mei_phy.o
obj-$(CONFIG_NFC_QNCI)		+= nfc-nci.o

#chenjian add for zx55q05 nfc start
obj-$(CONFIG_NFC_BCM2079X) 	+= bcm2079x-i2c.o
#chenjian add for zx55q05 nfc end

ccflags-$(CONFIG_NFC_DEBUG) := -DDEBUG
