#! /usr/bin/env python
#
#   BSD LICENSE
#
#   Copyright(c) 2016 Intel Corporation. All rights reserved.
#   All rights reserved.
#
#   Redistribution and use in source and binary forms, with or without
#   modification, are permitted provided that the following conditions
#   are met:
#
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in
#       the documentation and/or other materials provided with the
#       distribution.
#     * Neither the name of Intel Corporation nor the names of its
#       contributors may be used to endorse or promote products derived
#       from this software without specific prior written permission.
#
#   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
#   A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
#   OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
#   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
#   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
#   DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
#   THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
#   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
#   OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#

from __future__ import print_function
import sys
import os.path
import getopt
import subprocess

out = "build"
base = None
machine = None


def print_usage(cmd, f):
    print("""Usage: %s -b <base_config> [-o <output_dir>] [options...] """
          % os.path.basename(cmd), file=f)


def get_valid_configs():
    c = [f[10:] for f in os.listdir("config") if f.startswith("defconfig_")]
    c.sort()
    return c


def print_help(cmd):
    print_usage(cmd, sys.stdout)
    print("""
Generates a new DPDK build time configuration in the given output directory,
using as a starting point the provided base configuration. Once completed
successfully, DPDK can then be built using that configuration by running the
command "make -C <output_dir>".

-b, --base-config=CONFIG
    Use the configuration given by CONFIG as the starting point for the
    build configuration. CONFIG must be a valid DPDK default configuration
    provided by a file "defconfig_CONFIG" in the top level "config"
    directory.
    Valid --base-config values:\n\t\t%s

-o, --output-dir=DIR
    Use DIR as the resulting output directory. If unspecified the default
    value of "build" is used.

-m, --machine-type=TYPE
    Override the machine-type value given in the default configuration by
    setting it to TYPE. Using regular options, regular y/n values can be
    changed, but not string options, so this explicit override for machine
    type exists to allow changing it.

-h, --help
    Print this help text and then exit

Other options passed after these on the commandline should be in the format of
"name=y|n" and are overrides to be applied to the configuration to toggle the
y|n settings in the file.

Matches are applied on partial values, so for example "DEBUG=y" will turn on
all options ending in "DEBUG". This means that a full option need not always be
specified, for example, "CONFIG_RTE_LIBRTE_" prefixes can generally be omitted.

Examples:

To generate a configuration for DPDK
- based off x86_64-native-linuxapp-gcc,
- changing the machine type to "default",
- turning on the pcap PMD,
- disabling the KNI and igb_uio kernel modules, and
- placing the output in a suitably named folder
use:

    %s -o x86_64-default-linuxapp-gcc -b x86_64-native-linuxapp-gcc \\
        -m default PMD_PCAP=y IGB_UIO=n KNI_KMOD=n

""" % (",\n\t\t".join(get_valid_configs()), os.path.basename(cmd)))


def parse_opts(argv):
    global out, base, machine
    long_opts = ["output-dir", "base-config", "machine-type", "help"]
    try:
        opts, args = getopt.getopt(argv[1:], "o:b:m:h", long_opts)
    except getopt.GetoptError as err:
        print(str(err))
        print_usage(argv[0], sys.stderr)
        sys.exit(1)

    for o, a in opts:
        if o in ["-o", "--output-dir"]:
            out = a
        elif o in ["-b", "--base-config"]:
            base = a
        elif o in ["-m", "--machine-type"]:
            machine = a
        elif o in ["-h", "--help"]:
            print_help(argv[0])
            sys.exit()
        else:
            print("Error: Unhandled option '%s'\n" % o,
                  file=sys.stderr)
            sys.exit(1)

    if base is None:
        print_usage(argv[0], sys.stderr)
        sys.exit(1)
    if not os.path.exists("config/defconfig_%s" % base):
        print("Error, invalid base configuration specified: %s" % base)
        print("Valid cfgs: %s" % ",\n\t".join(get_valid_configs()))
        sys.exit(1)

    enable_args = [a[:-2] for a in args if a[-2:] == "=y"]
    disable_args = [a[:-2] for a in args if a[-2:] == "=n"]
    invalid = [a for a in args if a[-2:] not in ("=y", "=n")]
    if len(invalid) > 0:
        print("Error, options not ending in '=y' or '=n': %s"
              % str(invalid))
        sys.exit(1)
    return (enable_args, disable_args)


def run_make_config():
    try:
        # the flush parameter is not always implemented so provide fallback
        print("Running 'make config'...", end=' ', flush=True)
    except Exception:
        print("Running 'make config'...")

    cmd = "make config T=%s O=%s" % (base, out)
    if sys.platform.startswith("freebsd"):
        cmd = cmd.replace("make", "gmake", 1)
    if subprocess.call(cmd.split()) != 0:
        print("make config called failed")
        sys.exit(1)


def load_file(fn):
    fd = open(fn)
    contents = [line.strip() for line in fd.readlines()]
    fd.close()
    return contents


def write_file(fn, config):
    fd = open(fn, "w")
    fd.writelines(["%s\n" % line for line in config])
    fd.close()


def change_settings(enable, disable):
    config = load_file("%s/.config" % out)
    print("Enabling settings %s" % str(enable))
    for entry in enable:
        config = [line.replace("%s=n" % entry, "%s=y" % entry)
                  for line in config]
    print("Disabling settings %s" % str(disable))
    for entry in disable:
        config = [line.replace("%s=y" % entry, "%s=n" % entry)
                  for line in config]

    if machine is not None:
        print("Setting machine to '%s'" % machine)
        config.append("CONFIG_RTE_MACHINE=\"%s\"" % machine)
    write_file("%s/.config" % out, config)


def main():
    enable, disable = parse_opts(sys.argv)
    run_make_config()
    change_settings(enable, disable)
    print("Done")


if __name__ == "__main__":
    main()
