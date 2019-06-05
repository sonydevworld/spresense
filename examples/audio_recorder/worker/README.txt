
About postfilter worker
===========================

This directory is a build environment for postfilter worker.
You are able to build postfilter worker and implement as you like.
For example, low pass filter, band pass filter, and any more.

---------------------------
Directorys and files

-[lib]

   Including Makefile to build ASMP library which is used for inter CPU communication.
   Source codes are place at SDK directory, and you don't need to change them.
 
-[postfilter]

   Files for postfilter processing is here.
   Internaly, separete some directories.
 
  -[command]

     This directory includes header file which defines command formet of Postfilter worker.
     This command format is not need to change. User defined format could be written other file. (explain after.)

  -[framework]

     There are source codes for framework of Postfilter worker.
     Basically, you should not edit files. But you can edit too.
     If you edit these files, the operation cannot be guaranteed.

  -[userproc]

     Under this directory, all files are able edit by user freely.

     -include/userproc_packet_defs.h

       Command format defition.

     -include/userproc.h

       Header file of your source code. 

     -src/userproc.cxx

       Your source code. you can edit and write any process here.


 Postfilter source codes are there. Only templete is written. You are able to implement
 as you like.

---------------------------
Build

Postfilter worker build will be automatically run when build this(audio_player_pf) example.
When build complete "POSTFILTER" elf binary file will be created "postfilter" directory.


