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

  urg_library-X.X.X/vs2010(or vs2005)/c(or cpp)/urg.sln をビルドします。

  ビルド後は、urg.lib のスタティックライブラリと各サンプルの
  実行ファイルが生成されています。



Visual Studio bat compile (Windows)

＊以下の作業はコマンドプロンプト上で行ってください＊

  1. 環境変数を設定するために Visual Studio が提供している bat ファイルを
     コピーします。

  Microsoft Visual Studio 8/Common7/Tools/vsvars32.bat を
  urg-library-X.X.X/windowsexeにコピーする。


  2. 環境変数を設定後、コンパイル用のbatファイルを実行する。

  urg-library-X.X.X/windowsexe/compile.batを実行する。


  3. 生成されたサンプルの実行ファイルを動かす。

  urg-library-X.X.X/windowsexeに生成されるexeを実行する。


  4. 生成されたサンプルの実行ファイルを削除する。

  urg-library-X.X.X/windowsexe/cleanobj.batを実行し
  生成された実行ファイルを削除する。


gcc (Linux, MinGW)

  必要ならば urg_library-X.X.X/Makefile の PREFIX を編集して
  インストール先を変更します。

  !!! 現状こうなっているので、他の場所にしたければ、変更して下さい。
PREFIX = /usr/local
#PREFIX = /mingw

  コンパイルとインストールを行います。

  % make
  # make install

  ライブラリの使い方は、urg-library-X.X.X/samples/ 中の Makefile をご覧下さい。

  !!! ライブラリの使い方は、もう少しちゃんと書きたい。
