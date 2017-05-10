# MacReader

### See MasterThesis.pdf for full documentation

This is the git repository for a software defined RFID reader for USRP2.
This work is based on usrp2reader by Yuanqing Zheng which in turn is based on Gen2 project by Michael Buettner.

## Hardware:
	USRP N210, USRP RFX900 daughterboard  

The aim of the project was to create a setup capable of reproducing the findings of Buettner. 
It turned out that the software needed a major cleanup to be useable/maintainable by others, and was also based on deprecated technology, which resulted in a complete rewrite.
Not much of the original gen2/usrp2reader project is left.

The file MasterThesis.pdf fully documents how to use the software, how it was created, and everything in between.

## Abstract:
Many RFID protocol researchers today base their research on the 2010 gen2reader software from Buettner and Wetherall [7]. This software provides a partial network stack implementation of the EPC/RFID UHF Air Interface Protocol [20], which makes it an attractive method to save time on setup and quickly get started with the research. However, as this thesis shows, the gen2reader software has not been sufficiently tested itself and in fact contains several serious flaws that could affect the validity of measurements. In addition, the design and implementation of the software hinders extendability and modifiability of the gen2reader, and makes it difficult to reproduce results. This thesis first analyses the gen2reader software to identify these flaws, then eliminates them through a redesign that also makes the software much easier to extend and modify. The redesigned software, which we call the MacReader also clearly distinguishes the MAC and PHY layers. We validate the MacReader through PHY layer tests, which lead to several interesting results. We find that it is possible to affect the RFID tag response delay through manipulation of the RFID interrogator command preamble. We also identify a characteristic of the USRP N210 software-defined radio, that enables the MacReader to passively power the tag. Thereby it does not have to wait for the tag response falling edge, but instead can begin processing the response already from the first rising edge. We also profile the transmission power of the USRP N210’s daughterboard, and experiment with different methods for shielding the signal between the interrogator RX and TX antennas. Finally we identify a flaw in the WISP programmable RFID tag implementation that results in erroneous encoding of the tag responses. The value of this work is threefold; the exposal of the weaknesses of an established and accepted research platform, the proposal and implementation of a platform that addresses these weaknesses, and the performance of experiments that validate the platform and leads to several useful observations regarding the PHY layer and hardware behavior.
