The contents of this transaction was created by Padraic Garvey
of ARM on 29-Aug-2013.

It contains the ARM data versions listed below.

This data, unless otherwise stated, is ARM Proprietary and access to it
is subject to the agreements indicated below.

If you experience problems with this data, please contact ARM support
quoting transaction reference <551613>.

 - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

- DX910-SW-99002-r3p2-01rel3
  GPLv2 Linux Device Driver Source Code
  EAC Release
  Entitled via internal access

 - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

In addition to the data versions listed above, this transaction contains
three additional files at the top level.

The first is ARM_DELIVERY_551613.TXT, which is the delivery
note.

The second is ARM_MANIFEST_551613.TXT which contains a manifest of all the
files included in this transaction, together with their checksums.

The third is this file, README.TXT. It is necessary to include this file
to indicate the inclusion of the file :
	driver/src/devicedrv/mali/linux/mali_device_pause_resume.c 
in this part. A previous version of this part, DX910-SW-99002-r3p2-01rel3 
did not include the file mali_device_pause_resume.c, which is necessary to 
successfully build the opensource driver. This revision of the part
addresses that shortcoming.
The md5sum for the mali_device_pause_resume.c file is
bb7eb4c160e474bc2621b93d27a9d856.   

The checksums provided are calculated using the RSA Data Security, Inc.
MD5 Message-Digest Algorithm.

The checksums can be used to verify the integrity of this data using the
"md5sum" tool (which is part of the GNU "textutils" package) by running:

  % md5sum --check ARM_MANIFEST_551613.TXT
