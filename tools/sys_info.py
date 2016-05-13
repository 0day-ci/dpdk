#! /usr/bin/python
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

import os, sys
import getopt
import json
from os.path import exists, abspath, dirname, basename

version = "0.1.3"

# Global lists and flags
machine_readable = 0
show_pci = False
debug_flag = False
coremaps_flag = False

sys_info = {}
coremaps = {}

def proc_cpuinfo_path():
    '''Return the cpu information from /proc'''
    return "/proc/cpuinfo"

def proc_sysinfo_path():
    '''Return the system path string from /proc'''
    return "/proc/sysinfo"

def proc_meminfo_path():
    '''Return the memory information path from /proc'''
    return "/proc/meminfo"

def sys_system_path():
    '''Return the system path string from /sys'''
    return "/sys/devices/system"

def read_file(path, whole_file=False):
    '''Read the first line of a file'''

    if os.access(path, os.F_OK) == False:
        print "Path (%s) Not found" % path
        return ""

    fd = open(path)
    if whole_file == True:
        lines = fd.readlines()
    else:
        line = fd.readline()
    fd.close()

    if whole_file == True:
        return lines

    return line

def get_range(line):
    '''Split a line and convert to low/high values'''

    line = line.strip()

    if '-' in line:
        low, high = line.split("-")
    elif ',' in line:
        low, high = line.split(",")
    else:
        return [int(line)]

    return [int(low), int(high)]

def get_ranges(line):
    '''Split a set of ranges into first low/high, second low/high value'''

    line = line.strip()

    first, second = line.split(',')

    f = get_range(first)
    s = get_range(second)

    return [f, s]

def get_low(line):
    '''Return the low value in a range'''
    range = get_range(line)
    return int(range[0])

def get_high(line):
    '''Return the high value in a range'''
    range = get_range(line)

    if len(range) > 1:
        return range[1]

    return range[0]

def get_value(line):
    '''Convert a number into a integer value'''
    return int(line)

def cpu_path(name):
    '''Return the cpu path string to a given file name'''
    return "%s/cpu/%s" % (sys_system_path(), name)

def topology_path(id, name):
    '''Return the topology path string to a given file cpu id and file name'''
    return "%s/cpu/cpu%d/topology/%s" % (sys_system_path(), id, name)

def get_cpu_info(filename):
    ''' Read the file using cpu_base_path/name '''
    return read_file(cpu_path(filename))

def get_topology_info(id, filename):
    ''' Read the cpu_base_path/cpu(id)/topology/(name) file '''
    return read_file(topology_path(id, filename))

def cpulist_parse(info):
    ''' '''
    cpuset = []
    for c in range(0, 255):
        cpuset.append(0)

    ranges = info.split(',')
    for r in ranges:
        if not '-' in r:
            cpuset[int(r)] = 1
        else:
            low, high = get_range(r)
            for c in range(low, high + 1):
                cpuset[c] = 1

    return cpuset

def path_read_cpuset(fn, num):
    ''' '''
    info = get_topology_info(num, fn)
    return cpulist_parse(info.strip())

def set_count(arr):
    cnt = 0
    for c in arr:
        cnt = cnt + c
    return cnt

def read_topology(ncpus, cpu):
    global sys_info

    path = topology_path(cpu, "thread_siblings_list")
    if os.access(path, os.F_OK) == False:
        print "Path (%s) Not found" % path
        return ""

    thread_siblings = path_read_cpuset("thread_siblings_list", cpu)
    core_siblings = path_read_cpuset("core_siblings_list", cpu)

    path = topology_path(cpu, "book_siblings_list")
    if os.access(path, os.F_OK):
        book_siblings = path_read_cpuset("book_siblings_list", cpu)
    else:
        book_siblings = []

    if not 'nthreads' in sys_info:
        nthreads = set_count(thread_siblings)
        if nthreads == 0:
            nthreads = 1
        ncores = set_count(core_siblings)/nthreads
        if ncores == 0:
            ncores = 1
        nsockets = ncpus / nthreads / ncores
        if nsockets == 0:
            nsockets = 1
        nbooks = ncpus / nthreads / ncores / nsockets
        if nbooks == 0:
            nbooks = 1
        nlcores = nbooks * nsockets * ncores * nthreads
        sys_info['nthreads'] = nthreads
        sys_info['ncores'] = ncores
        sys_info['nsockets'] = nsockets
        sys_info['nbooks'] = nbooks
        sys_info['nlcores'] = nlcores

def get_core_ids():
    ''' Return the core ids and the lcores for each socket assigned
    to the core id '''

    core_ids = []

    # Create cpu id to lcore id list core_ids[lcore] == core_id
    # if the core_id does not exist in the list already
    for c in range(0, sys_value('nlcores')):
        id = get_value(get_topology_info(c, "core_id"))
        if id not in core_ids:
            core_ids.append(id)

    return core_ids

def get_coremaps():
    ''' Return the core ids and the lcores for each socket assigned
    to the core id '''
    global sys_info

    coremaps = {}

    for c in range(0, sys_value('nlcores')):
        id      = int(get_topology_info(c, "core_id").strip())
        socket  = int(get_topology_info(c, "physical_package_id").strip())
        ranges  = get_range(get_topology_info(c, "thread_siblings_list"))

        if coremaps.has_key(id):
            if not socket in coremaps[id]:
                coremaps[id].update({socket: ranges})
        else:
            coremaps[id] = {socket: ranges}

    return coremaps

def get_model_name():
    '''Parse up the CPU information into tables. The information
    is taken from /proc/cpuinfo file. '''
    model = "Unknown"

    if os.access(proc_cpuinfo_path(), os.F_OK) == False:
        return details

    lines = read_file(proc_cpuinfo_path(), True)

    for line in lines:
        line = line.strip()
        if len(line) != 0:
            name, value = line.split(":", 1)
            name = name.strip()
            value = value.strip()
            if 'model name' == name:
                return value
        else:
            break

    return model

def parse_meminfo():
    ''' Parse up the /proc/meminfo if available and return a dictionary. '''
    details = {}

    if os.access(proc_meminfo_path(), os.F_OK) == False:
        return details;

    lines = read_file(proc_meminfo_path(), True)

    for line in lines:
        if len(line.strip()) != 0:
            name, value = line.split(":", 1)
            details[name.strip()] = value.strip()

    return details

def sys_value(key):
    '''Get a value from the System Info dictionary if available. '''
    global sys_info

    if key in sys_info:
        return sys_info[key];

    for core in sys_info:
        if key in core:
            return core[key]

    return "Not Found"

def mem_value(key):
    '''Get a value from the Memory Info dictionary if available. '''

    info = sys_info['memory']

    if key in info:
        return info[key]

    return "Not Found"

def get_ether_devices():
    ''' Read the PCI ethernet list into a table '''

    fd = os.popen("lspci | grep Ether", 'r')
    lines = fd.readlines()
    fd.close()

    dict = {}
    for line in lines:
        pci, desc = line.rsplit(':',1)
        pci_id, _ = pci.split(' ', 1)
        if not 'Null' in desc:
            dict[pci_id] = desc.strip();

    return dict

def format_cores(val):
    '''Format a list of lcores into '[%3d,%3d,...]' '''
    s = "["
    for r in val:
        s = s + "%3d," % r
    s = s[:-1]
    s = s + "]"
    return s

def print_core_ids(ids):
    '''Return the string of a set of lcores for each id'''
    s = ""
    for i in ids:
        s = s + format_cores(ids[i]) + "   "

    return s

def print_lcore_pairs():
    ''' Print out the header line for the socket/lcore data'''

    nsockets = sys_value('nsockets')

    print "\nShowing mapping of lcores to core_ids and sockets:"
    print " Core ID ",
    for id in range(0, nsockets):
        print "  Socket %d " % id,
    print ""

    print " -------",
    for id in range(0, nsockets):
        print "  ---------",
    print ""

    for id in sys_info['core_ids']:
        vals = sys_info['coremaps']
        print "   %3d     %s" % (id, print_core_ids(vals[id]))

def usage():
    '''Print usage information for the program'''
    print("""
Usage: %s [options]

Show the lcores layout with core id and socket(s).

Options:
    --help or -h:
        Display the usage information and quit

    --long or -l:
        Display the information in a machine readable long format.

    --short or -s:
        Display the information in a machine readable short format.

    --pci or -p:
        Display all of the Ethernet devices in the system using 'lspci'.

    --version or -v:
        Display the current version number of this script.

    --debug or -d:
        Output some debug information.

    default:
        Display the information in a human readable format.

    """ % basename(sys.argv[0]))

def parse_args():
    '''Parse the command line arguments'''
    global machine_readable
    global show_pci
    global debug_flag
    global coremaps_flag
    global args

    if len(sys.argv) == 1:
        return
    try:
        opts, args = getopt.getopt(sys.argv[1:], "hslpvdc",
                   ["help", "short", "long", "pci", "version", "debug", "coremaps"])

    except getopt.GetoptError as error:
        print "** Error ", str(error)
        usage()
        sys.exit(1)

    machine_readable = 0
    for opt, arg in opts:
        if opt == "--help" or opt == "-h":
            usage()
            sys.exit(0)
        if opt == "--short" or opt == "-s":
            machine_readable = 1
        if opt == "--long" or opt == "-l":
            machine_readable = 2
        if opt == "--pci" or opt == "-p":
            show_pci = True
        if opt == "--debug" or opt == "-d":
            debug_flag = True
        if opt == "--coremaps" or opt == "-c":
            coremaps_flag = True
        if opt == "--version" or opt == "-v":
            print "%s version %s" % (sys.argv[0], version)
            sys.exit(0)

def gather_information():
    global sys_info

    sys_info['memory'] = parse_meminfo()

    sys_info.update({'model name': get_model_name()})

    ncpus = get_high(get_cpu_info("present")) + 1
    for cpu in range(0, ncpus):
        read_topology(ncpus, cpu)

    uname = os.uname()
    sys_info.update({'sysname': uname[0]})
    sys_info.update({'release': uname[2]})
    sys_info.update({'version': uname[3]})
    sys_info.update({'machine': uname[4]})

    sys_info['core_ids'] = get_core_ids()
    sys_info['coremaps'] = get_coremaps()

    if show_pci:
        info = get_ether_devices()
        sys_info['pci'] = info
    else:
        sys_info['pci'] = {}

    if debug_flag == True:
        print "sys_info = ", json.dumps(sys_info, indent=4)

def main():
    '''program main function'''
    global machine_readable
    global sys_info

    parse_args()

    gather_information()

    if machine_readable > 0:
        json_details = {'system': {
            'sysname':          sys_info['sysname'],
            'version':          sys_info['version'],
            'release':          sys_info['release'],
            'machine':          sys_info['machine'],
            }}

        cpu_info = {'cpu': {
            'model_name':       sys_value('model name'),
            'nthreads':         sys_value('nthreads'),
            'ncores':           sys_value('ncores'),
            'nsockets':         sys_value('nsockets'),
            'nbooks':           sys_value('nbooks'),
            'nlcores':          sys_value('nlcores'),
            'cores_ids':        sys_value('core_ids')}}

        memory = {'memory': {
            'total':        mem_value('MemTotal'),
            'free':         mem_value('MemFree'),
            'available':    mem_value('MemAvailable')}}
        hugepages = {'hugepages': {
            'total':        mem_value('HugePages_Total'),
            'free':         mem_value('HugePages_Free'),
            'reserved':     mem_value('HugePages_Rsvd'),
            'surp':         mem_value('HugePages_Surp'),
            'page_size':    mem_value('Hugepagesize')}}

        json_details.update(cpu_info)
        json_details.update(memory)
        json_details.update(hugepages)
        json_details.update({'coremaps': sys_value('coremaps')})
        if show_pci:
            json_details.update({'pci': sys_value('pci')})

        if machine_readable == 2:
            print json.dumps(json_details, sort_keys=False, indent=4)
        else:
            print json.dumps(json_details, sort_keys=False, separators=(', ', ':'))
    else:
        if len(sys_info) > 0:
            print "System Information:"
            print "   Model Name : %s" % sys_value('model name')
            print "   sysname    : %s" % sys_value('sysname')
            print "   release    : %s" % sys_value('release')
            print "   version    : %s" % sys_value('version')
            print "   machine    : %s" % sys_value('machine')
            print ""

            print "CPU Information:"
            print "   nthreads   : %3d" % sys_value('nthreads')
            print "   ncores     : %3d" % sys_value('ncores')
            print "   nsockets   : %3d" % sys_value('nsockets')
            print "   nbooks     : %3d" % sys_value('nbooks')
            print "   nlcores    : %3d" % sys_value('nlcores')

            print ""
        if len(sys_info['memory']) > 0:
            print "Memory:"
            print "   Total    : ", mem_value('MemTotal')
            print "   Free     : ", mem_value('MemFree')
            print "   Available: ", mem_value('MemAvailable')
            print ""
            print "Huge Pages:"
            print "   Total    : ", mem_value('HugePages_Total')
            print "   Free     : ", mem_value('HugePages_Free')
            print "   Reserved : ", mem_value('HugePages_Rsvd')
            print "   Surp     : ", mem_value('HugePages_Surp')
            print "   Page Size: ", mem_value('Hugepagesize')

        if show_pci:
            print "\nList of devices labeled as Ethernet on the PCI bus"
            print "   PCI ID   Description"
            info = sys_value('pci')
            for p in info:
                print "  %s   %s" % (p, info[p])

        print_lcore_pairs()

        vals = (sys_value('nlcores'), sys_value('nsockets'), sys_value('ncores'), sys_value('nthreads'))
        print "\n%d lcores, %d socket(s), %d cores per socket, %d hyper-thread(s) per core" % vals

if __name__ == "__main__":
    main()
