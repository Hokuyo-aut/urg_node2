URG Library

About this software:
  This software have been developed to provide a library to use
  scanning range sensors of Hokuyo Automatic Corporation.  Samples
  will help to know how to use them.

Authors:
  Satofumi Kamimura <satofumi@users.sourceforge.net> Design and implementation of the first version.
  Katsumi Kimoto

License:
  (C)Simplified BSD License.
  (C++)LGPL License.
  See COPYRIGHT file.


Mailing list:
  urgnetwork-support@lists.sourceforge.net


Library usage:

Visual Studio Solution (Windows)	

  Use urg_library-X.X.X/vs2010(or vs2005)/c(or cpp)/urg.sln project file to build.

  After building, the urg.lib static library file and the executable files for each
  sample program will be generated.


Visual Studio bat compile (Windows)
*Perform the following steps using the command prompt*

  1. To configure the environment variables, copy the batch (.bat) file provided by Visual Studio.
  Copy the following file:
    Microsoft Visual Studio 8/Common7/Tools/vsvars32.bat
  into the folder:
    urg-library-X.X.X/windowsexe

  2. After configuring the environment variables (the above .bat file), execute the following .bat
  file to compile:
    urg-library-X.X.X/windowsexe/compile.bat

  3. The executable file for the sample programs is generated.
  The samples' executable files are found in the folder:
    urg-library-X.X.X/windowsexe

  4. To clean up generated executable files run the following .bat file:
    urg-library-X.X.X/windowsexe/cleanobj.bat


gcc (Linux, MinGW)

  If you need to change the default installation directory, change the PREFIX variable
  in the following file:
     urg_library-X.X.X/current/Makefile.release

  This is the default setting, change it to suit your needs:
PREFIX = /usr/local
#PREFIX = /mingw

  To compile and install, perform the following steps:
  $ make
  # make install

  To learn how to use the library, see the Makefile and source codes on the following folder:
    urg-library-X.X.X/samples/ 

