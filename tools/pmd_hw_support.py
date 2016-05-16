#!/usr/bin/python3
#-------------------------------------------------------------------------------
# scripts/pmd_hw_support.py
#
# Utility to dump PMD_INFO_STRING support from an object file
#
#-------------------------------------------------------------------------------
import os, sys
from optparse import OptionParser
import string
import json

# For running from development directory. It should take precedence over the
# installed pyelftools.
sys.path.insert(0, '.')


from elftools import __version__
from elftools.common.exceptions import ELFError
from elftools.common.py3compat import (
        ifilter, byte2int, bytes2str, itervalues, str2bytes)
from elftools.elf.elffile import ELFFile
from elftools.elf.dynamic import DynamicSection, DynamicSegment
from elftools.elf.enums import ENUM_D_TAG
from elftools.elf.segments import InterpSegment
from elftools.elf.sections import SymbolTableSection
from elftools.elf.gnuversions import (
    GNUVerSymSection, GNUVerDefSection,
    GNUVerNeedSection,
    )
from elftools.elf.relocation import RelocationSection
from elftools.elf.descriptions import (
    describe_ei_class, describe_ei_data, describe_ei_version,
    describe_ei_osabi, describe_e_type, describe_e_machine,
    describe_e_version_numeric, describe_p_type, describe_p_flags,
    describe_sh_type, describe_sh_flags,
    describe_symbol_type, describe_symbol_bind, describe_symbol_visibility,
    describe_symbol_shndx, describe_reloc_type, describe_dyn_tag,
    describe_ver_flags,
    )
from elftools.elf.constants import E_FLAGS
from elftools.dwarf.dwarfinfo import DWARFInfo
from elftools.dwarf.descriptions import (
    describe_reg_name, describe_attr_value, set_global_machine_arch,
    describe_CFI_instructions, describe_CFI_register_rule,
    describe_CFI_CFA_rule,
    )
from elftools.dwarf.constants import (
    DW_LNS_copy, DW_LNS_set_file, DW_LNE_define_file)
from elftools.dwarf.callframe import CIE, FDE

raw_output = False;

class ReadElf(object):
    """ display_* methods are used to emit output into the output stream
    """
    def __init__(self, file, output):
        """ file:
                stream object with the ELF file to read

            output:
                output stream to write to
        """
        self.elffile = ELFFile(file)
        self.output = output

        # Lazily initialized if a debug dump is requested
        self._dwarfinfo = None

        self._versioninfo = None

    def _section_from_spec(self, spec):
        """ Retrieve a section given a "spec" (either number or name).
            Return None if no such section exists in the file.
        """
        try:
            num = int(spec)
            if num < self.elffile.num_sections():
                return self.elffile.get_section(num)
            else:
                return None
        except ValueError:
            # Not a number. Must be a name then
            return self.elffile.get_section_by_name(str2bytes(spec))

    def parse_pmd_info_string(self, mystring):
        global raw_output
        i = mystring.index("=");
        mystring = mystring[i+2:]
        pmdinfo = json.loads(mystring)

        if raw_output:
            print(pmdinfo)
            return

        print("PMD NAME: " + pmdinfo["name"])
        print("PMD TYPE: " + pmdinfo["type"])
        if (pmdinfo["type"] == "PMD_PDEV"):
            print("PMD HW SUPPORT:")
            print("VENDOR\t DEVICE\t SUBVENDOR\t SUBDEVICE")
            for i in pmdinfo["pci_ids"]:
                print("0x%04x\t 0x%04x\t 0x%04x\t\t 0x%04x" % (i[0], i[1], i[2], i[3]))

        print("")


    def display_pmd_info_strings(self, section_spec):
        """ Display a strings dump of a section. section_spec is either a
            section number or a name.
        """
        section = self._section_from_spec(section_spec)
        if section is None:
            return


        found = False
        data = section.data()
        dataptr = 0

        while dataptr < len(data):
            while ( dataptr < len(data) and
                    not (32 <= byte2int(data[dataptr]) <= 127)):
                dataptr += 1

            if dataptr >= len(data):
                break

            endptr = dataptr
            while endptr < len(data) and byte2int(data[endptr]) != 0:
                endptr += 1

            found = True
            mystring = bytes2str(data[dataptr:endptr])
            rc = mystring.find("PMD_INFO_STRING")
            if (rc != -1):
                self.parse_pmd_info_string(mystring)

            dataptr = endptr


def main(stream=None):
    global raw_output

    optparser = OptionParser(
            usage='usage: %prog [-h|-r] <elf-file>',
            description="Dump pmd hardware support info",
            add_help_option=True,
            prog='pmd_hw_support.py')
    optparser.add_option('-r', '--raw',
            action='store_true', dest='raw_output',
            help='Dump raw json strings')

    options, args = optparser.parse_args()

    if options.raw_output:
        raw_output = True

    with open(args[0], 'rb') as file:
        try:
            readelf = ReadElf(file, stream or sys.stdout)
   
            readelf.display_pmd_info_strings(".rodata") 
            sys.exit(0)
 
        except ELFError as ex:
            sys.stderr.write('ELF error: %s\n' % ex)
            sys.exit(1)


#-------------------------------------------------------------------------------
if __name__ == '__main__':
    main()


