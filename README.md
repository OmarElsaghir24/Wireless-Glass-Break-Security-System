# Wireless-Glass-Break-Security-System
Implemented a wireless glass break security system with the TM4C123GH6PM microcontroller that goes into hibernation mode for a duration of time of the user's preferences. Designed a PCB with battery-backed capabilities to power the microcontroller, and a voltage divider to reduce the voltage to a value at a safe level for ADC (Analog-to-Digital Converter) converter to measure voltage left. A packet is sent using the nrf24l01+ wireless module from the microcontroller to a remote server after the TM4C123GH6PM wakes up out of hibernation. Packet details include battery level of the battery pack, if the device is a low power device, and if the microcontroller woke up from hibernation due to an alarm or if an object struck the glass.

![Wireless Glass Break Security System](https://github.com/user-attachments/assets/150c1e41-db03-4cd0-982b-2e4dff8f865e)

# Features
* `nrf24l01+ wireless module`: used to transmit to and receive packets from a remote server. Goes through a 3-way handshake to establish connection with remote server. Microcontroller communicates with nrf24l01+ through SPI interface.
* `Wake-up from external event`: microcontroller wakes up from hibernation when an object strikes the piezoelectric sensor attached to glass.
* `Alarm-triggered wake-up event`: microcontroller wakes up from hibernation when an alarm time has been reached. Implemented using the Hibernation module.
* `Low Battery wake-up event`: microcontroller wakes up from hibernation due to low power from battery pack.

# Hardware Components
|                           |
|---------------------------|
| TM4C123GH6PM Tiva Board |
| nrf24l01+ wireless module |
| Piezoelectric sensor |
| Battery pack |
| 1.25 Volt batteries (4) |

# Circuit Diagram

![IoT Project 2 Voltage Divider](https://github.com/user-attachments/assets/b337d8a1-a094-4ea0-8f0d-15d64bed98fd)

# Peripherals Used
|                   |
|-------------------|
| GPIO |
| Timers |
| SPI |
| Hibernation |
| ADC0 |
