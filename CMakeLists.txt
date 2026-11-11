# esp32p4-ntp — step 1: Ethernet bring-up (RMII + IP101 PHY), DHCP, ping.
cmake_minimum_required(VERSION 3.16)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(esp32p4_ntp)
