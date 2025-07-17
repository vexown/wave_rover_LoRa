# SDR (Software Defined Radio) is a technology that allows software to perform radio signal processing tasks.
#
# For analysis of sub-GHz LoRa signals I recommend using the RTL-SDR USB dongle.
#
# Getting the hardware:
# Official RTL-SDR seller on Aliexpress (it's their official store and much cheaper than buying from Polish sellers): https://www.rtl-sdr.com/buy-rtl-sdr-dvb-t-dongles/
# Store name: RTLSDRBlog Store (4523039)
# Link to the product: https://pl.aliexpress.com/item/1005005952566458.html?spm=a2g0o.order_list.order_list_main.11.c7f01c24L0XenX&gatewayAdapt=glo2pol
#
# Setting up:
# Go to https://www.rtl-sdr.com/rtl-sdr-quick-start-guide/ and follow the instructions for your operating system (Linux for my case)
# In terms of software, I have been using SDRPlusPlus - to install it follow the instructions on their official GitHub page:
# https://github.com/AlexandreRouma/SDRPlusPlus 
#
# Then just plug in the RTL-SDR dongle and run this script to start SDRPlusPlus to observe your sub-GHz signals (in this case LoRa):
sdrpp

