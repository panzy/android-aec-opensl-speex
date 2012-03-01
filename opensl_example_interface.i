%module opensl_example
%{
    #include <SLES/OpenSLES.h>
    #include <SLES/OpenSLES_Android.h>
    #include "opensl_io.h"
    #include "opensl_example.h"
%}

// Enable the JNI class to load the required native library.
%pragma(java) jniclasscode=%{
  static {
    try {
        java.lang.System.loadLibrary("opensl_example");
    } catch (UnsatisfiedLinkError e) {
        java.lang.System.err.println("native code library failed to load.\n" + e);
        java.lang.System.exit(1);
    }
  }
%}

%include "opensl_example.h"
