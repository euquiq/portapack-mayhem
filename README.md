I own a chinese flavored H1 Portapack. This particular fork is finetuned for this model, although it works perfectly OK (AFAIK) on H2 models.

This repository is my coding "buffer" before pulling requests into eried/portapack-mayhem repository, which represents at this time and date one of the main efforts (if not the only one) to keep fixing and further developing the HAVOC version, now re-bautized into "MAYHEM"

## Version highlights
* My version is H1 (clone) centered: It includes a sensitivity selector for the touchscreen, and understands the difference between the headphones and speaker outputs (You should install a speaker!). Also, it does NOT include any enhancement from Mayhem that -according to my sometimes subjective criteria- goes against the overall good functioning of the device. 

### Some differences (useful for H1 users)

* Two new checkboxes on OPTIONS -> UI menu:

The first checkbox adds an extra menu button on the upper left position of every sub-menu, ".." allowing for easy going back into the parent menu. This comes from the difficulty of pressing the small arrow on the status view at top of the lcd screen, specially with the mediocre sensitivity of the resistive touchscreen sensor included on some chinese portapack versions.

The second checkbox enables a speaker control icon on the status bar. You may be aware that your Portapack allows for speaker connection, but it is not included in the hardware, as is. So you can solder a small tablet - laptop speaker (i.e. 8 ohms / 1 watt) into the speaker header / solder pads of your portapack. You can turn on / off the speaker by toggling the speaker control icon.

* The DEBUG -> PERIPHERALS submenu is now also buttons-based.

* DEBUG, DEBUG -> PERIPHERALS and OPTIONS buttons menu is 1 button per row

* Touchscreen can have its sensitivity enhanced (r_touch_threshold lowered). You can change it on the calibration screen.

* The speaker icon on taskbar, when green, activates internal speaker (and mutes headphones output). When greyed out, internal speaker is muted, and headphones output is activated (this is useful only for H1 users)
