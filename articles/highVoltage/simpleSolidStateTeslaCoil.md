---
layout: article
title: Simple Solid State Tesla Coil
---

This article is about a simple solid state Tesla coil which I built a few years ago. I do not recommend rebuilding it and do not provide building instructions here. High voltage can be dangerous. The Tesla coil produces high voltage in the range of several $$\mathrm{kV}$$ and a frequency of several hundred $$\mathrm{kHz}$$. The power stage of this Tesla coil is feed by the rectified output of a variac which produces up to $$250\,\mathrm{VAC}$$ at its output. Besides that, the Tesla coil can cause electromagnetic interference problems and may even damage sensitive electronic devices. 

<figure>
    <img src="/assets/images/sstc_sparks.png" alt="MISSING IMAGE" style="width:100%"/>
    <figcaption></figcaption>
</figure>


## Circuit and Setup

The circuit is indeed very simple. On the left side, there is a tunable oscillator based on the well-known 555 timer chip. Its frequency can be set via a potentiometer. The output of the oscillator is feed into a driver stage built with [BD139](https://pdf1.alldatasheet.com/datasheet-pdf/view/16171/PHILIPS/BD139.html)/[BD140](https://pdf1.alldatasheet.com/datasheet-pdf/view/16172/PHILIPS/BD140.html) BJTs. The output of this driver stage is connected to a self wound gate drive transformer which drives a half bridge consisting of two [IRF740](https://www.vishay.com/docs/91054/91054.pdf) power MOSFETs.

<figure>
    <img src="/assets/images/sstc_board.JPG" alt="MISSING IMAGE" style="width:100%"/>
    <figcaption></figcaption>
</figure>

The complete setup can be seen in the picture below. The oscillator/driver stage is supplied with $$12\,\mathrm{V}$$ by my lab bench power supply. The output of this supply is floating, which is crucial since the power stage, i.e. the MOSFET half bridge, is supplied via the rectified output of a variac whose primary is directly connected to mains voltage. This setup, besides being very dangerous, puts a lot of stress on the internal isolation of the lab PSU and may damage it. I do not recommend such a setup by any means. Always use an isolation transformer when working with variacs!

<figure>
    <img src="/assets/images/sstc_setup.JPG" alt="MISSING IMAGE" style="width:100%"/>
    <figcaption></figcaption>
</figure>

Before I built this Tesla coil, I built a similar one but with an [IR2104](https://www.infineon.com/dgdl/Infineon-IR2104-DS-v01_00-EN.pdf?fileId=5546d462533600a4015355c7c1c31671) integrated half bridge driver instead of a gate drive transformer (instead of using the IR2104, I would now use the [IR2184](https://www.infineon.com/dgdl/Infineon-IR2184(4)(S)-DataSheet-v01_00-EN.pdf?fileId=5546d462533600a4015355c955e616d4), which has the same pinout as the IR2104 but drives the MOSFETs with higher gate currents), see the following picture.

<figure>
    <img src="/assets/images/sstc_ir2104.JPG" alt="MISSING IMAGE" style="width:100%"/>
    <figcaption></figcaption>
</figure>
    


## Test Results

The following video shows the Tesla coil in action. The size of the spark varies as I tune the frequency of the oscillator. I am pleased with the output of this Tesla coil, though I think there is a lot of room for improvements. Producing sparks of this size should easily be possible with much lower supply voltages of e.g. $$24\,\mathrm{V}$$ instead of the rectified output of a variac. You may want to check out the YouTube channel [Teslaundmehr](https://www.youtube.com/user/Teslaundmehr). There you can find solid state Tesla coils producing sparks as long as the secondary itself.

<video controls class="center">
    <source src="/assets/images/sstc_sparks.MOV" type="video/mp4">
</video>