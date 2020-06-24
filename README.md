# ESP32 2-Way Audio Relay
 ESP32-based 2-way audio interface for wireless voice and data relay between a network terminal and an amateur radio.
 
 For use with VBAN Banana running on a host PC.
 https://www.vb-audio.com/Voicemeeter/banana.htm
 
 The ESP32 samples analog audio and streams to host PC over Wifi, using VBAN protocol, and converts network audio stream into analog audio.  The ESP32 also provides a control signal for an external open-drain push-to-talk circuit.
 
 When the ESP32 is wired to a ham radio, the ESP32 wirelessly relays audio from the radio to a host PC and vice-versa, enabling 2-way voice and digital packet modes with no physical connection between the radio and a host PC.  This has been tested with both Winlink and APRS through a Baofeng UV-5R.

 https://hackaday.io/project/170710-esp32-tnc-and-audio-relay-for-hfvhf-packet-radio
