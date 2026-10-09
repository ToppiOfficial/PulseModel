# KeyValues3

C++ text and binary KV3 reader with text output. Binary layout and AG2 skeleton conversion were studied against ValveResourceFormat: https://github.com/ValveResourceFormat/ValveResourceFormat. Its MIT notice is retained in ValveResourceFormat.LICENSE.

Binary KV3 versions 1-5 are implemented; real validation currently covers CS2 version 5 resources and compiler outputs. Zstandard decompression uses the vendored BSD-3-Clause option. The C++ LZ4 block decoder is local code and needs further review against an established library.

Also holds the Source 2 resource DATA-block helpers, since that block is binary KV3. No .NET runtime is used.
