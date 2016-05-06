#! /usr/bin/python3

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
# This script maps the set of pipelines identified (MASTER pipelines are 
# ignored) from the input configuration file to the set of cores 
# provided as input argument and creates configuration files for each of 
# the mapping combinations.
#

import sys
import array
import itertools
import re
import argparse
import os
from collections import namedtuple

#default values
enable_stage0_traceout = 1
enable_stage1_traceout = 1
enable_stage2_traceout = 1

#enable_stage0_fileout = 0
enable_stage1_fileout = 1
enable_stage2_fileout = 1

#pattern for physical core
pattern_phycore = '^(s|S)\d(c|C)[1-9][0-9]*$'
reg_phycore = re.compile(pattern_phycore)

#----------------------------------------------------------------------------- 
def popcount(mask):                                                                          
    return bin(mask).count("1")

#----------------------------------------------------------------------------- 
def len2mask(length):
    if (length == 0):
        return 0

    if (length > 64):
        sys.exit('error: len2mask - lenght %i > 64. exiting' %length)
    
    return (0xFFFFFFFFFFFFFFFF >> (64 - length))

#----------------------------------------------------------------------------- 
def bitstring_write(n, n_bits):
    tmpstr = ""
    if (n_bits > 64):
        return

    i = n_bits - 1
    while (i >= 0):
        cond = (n & (1 << i))
        if (cond):
            print('1', end='')
            tmpstr += '1'
        else:
            print('0', end='')
            tmpstr += '0'
        i -= 1
    #end while
    return tmpstr
#end function

#------------------------------------------------------------------------- 
Constants = namedtuple('Constants', ['MAX_CORES', 'MAX_PIPELINES'])
constants = Constants(16, 64)


#------------------------------------------------------------------------- 
class Cores0:
    def __init__(self):
        self.n_pipelines = 0

class Cores1:
    def __init__(self):
        self.pipelines = 0
        self.n_pipelines = 0

class Cores2:
    def __init__(self):
        self.pipelines = 0
        self.n_pipelines = 0
        self.counter = 0
        self.counter_max = 0
        self.bitpos = array.array("L", itertools.repeat(0,constants.MAX_PIPELINES))


#--------------------------------------------------------------------
class Context0:
    def __init__(self):
        self.cores = [Cores0() for i in range(0, constants.MAX_CORES)]
        self.n_cores = 0
        self.n_pipelines = 0
        self.n_pipelines0 = 0
        self.pos = 0
        self.file_comment = ""
        self.c1 = None
        self.c2 = None

    #-------------------------------------------------------------
    def stage0_print(self):
        print('printing Context0 obj')
        print('c0.cores(n_pipelines) = [ ', end='')
        for cores_count in range(0, constants.MAX_CORES):
            print(self.cores[cores_count].n_pipelines, end=' ')
        print(']')
        print('c0.n_cores = %d' %self.n_cores)
        print('c0.n_pipelines = %d' %self.n_pipelines)
        print('c0.n_pipelines0 = %d' %self.n_pipelines0)
        print('c0.pos = %d' %self.pos)
        print('c0.file_comment = %s' %self.file_comment)
        if (self.c1 is not None):
            print('c0.c1 = ', end='')
            print(repr(self.c1))
        else:
            print('c0.c1 = None')

        if (self.c2 is not None):
            print('c0.c2 = ', end='')
            print(repr(self.c2))
        else:
            print('c0.c2 = None')

                    
    #-------------------------------------------------------------
    def stage0_init(self, num_cores, num_pipelines, c1, c2):
        self.n_cores = num_cores
        self.n_pipelines = num_pipelines
        self.c1 = c1
        self.c2 = c2

    #-------------------------------------------------------------
    def stage0_process(self):
        #print('inside stage0_process')

        # stage0 init
        self.cores[0].n_pipelines = self.n_pipelines
        self.n_pipelines0 = 0
        self.pos = 1

        while True:
            #go forward
            while True:
                if ((self.pos < self.n_cores) and (self.n_pipelines0 > 0)):
                    self.cores[self.pos].n_pipelines = min(self.cores[self.pos - 1].n_pipelines, self.n_pipelines0)
                    self.n_pipelines0 -= self.cores[self.pos].n_pipelines
                
                    self.pos += 1
                else:
                    break

            #end while
            
            # check solution
            if (self.n_pipelines0 == 0):
                self.stage0_log()
                self.c1.stage1_init(self, self.c2) # self is object c0
                self.c1.stage1_process()
        
            # go backward
            while True:
                if (self.pos == 0):
                    return

                self.pos -= 1
                if ((self.cores[self.pos].n_pipelines >1) and
                        (self.pos != (self.n_cores -1))):
                    break
                
                self.n_pipelines0 += self.cores[self.pos].n_pipelines
                self.cores[self.pos].n_pipelines = 0
            #end while

            # rearm
            self.cores[self.pos].n_pipelines -= 1
            self.n_pipelines0 += 1
            self.pos += 1
        #end while         
    #end function    


    #-------------------------------------------------------------
    def stage0_log(self):
        tmp_file_comment = ""
        if(enable_stage0_traceout != 1):
            return

        print('STAGE0: ', end='')
        tmp_file_comment += 'STAGE0: '
        for cores_count in range(0, self.n_cores):
            print('C%d = %d\t' \
            %(cores_count, self.cores[cores_count].n_pipelines), end='')
            tmp_file_comment += "C{} = {}\t".format(cores_count, \
                    self.cores[cores_count].n_pipelines)
        #end for
        print('')
        self.c1.stage0_file_comment = tmp_file_comment
        self.c2.stage0_file_comment = tmp_file_comment

    # end function

#end class Context0


#-------------------------------------------------------------
class Context1:
    #class attribute
    _fileTrace = None

    def __init__(self):
        self.cores = [Cores1() for i in range(constants.MAX_CORES)]
        self.n_cores = 0
        self.n_pipelines = 0
        self.pos = 0
        self.stage0_file_comment = ""
        self.stage1_file_comment = ""

        self.c2 = None
        self.arr_pipelines2cores = []
    #end init

    #-------------------------------------------------------------
    def stage1_reset(self):
        for i in range(constants.MAX_CORES):
            self.cores[i].pipelines = 0
            self.cores[i].n_pipelines = 0
        
        self.n_cores = 0
        self.n_pipelines = 0
        self.pos = 0
        self.c2 = None
        self.arr_pipelines2cores.clear()
    #end def stage1_reset

    #-------------------------------------------------------------
    def stage1_print(self):
        print('printing Context1 obj')
        print('c1.cores(pipelines,n_pipelines) = [ ', end='')
        for cores_count in range(0, constants.MAX_CORES):
            print('(%d,%d)' %(self.cores[cores_count].pipelines, 
                              self.cores[cores_count].n_pipelines), end=' ')
        print(']')
        print('c1.n_cores = %d' %self.n_cores)
        print('c1.n_pipelines = %d' %self.n_pipelines)
        print('c1.pos = %d' %self.pos)
        print('c1.stage0_file_comment = %s' %self.stage0_file_comment)
        print('c1.stage1_file_comment = %s' %self.stage1_file_comment)
        if (self.c2 is not None):
            print('c1.c2 = ', end='')
            print(self.c2)
        else:
            print('c1.c2 = None')
    #end stage1_print

    #-------------------------------------------------------------
    def stage1_init(self, c0, c2):
        self.stage1_reset()  
        self.n_cores = 0
        while (c0.cores[self.n_cores].n_pipelines > 0):
            self.n_cores += 1
        
        self.n_pipelines = c0.n_pipelines
        self.c2 = c2

        self.arr_pipelines2cores = [0] * self.n_pipelines

        i = 0
        while (i < self.n_cores):
            self.cores[i].n_pipelines = c0.cores[i].n_pipelines
            i += 1
        #end while
    #end stage1_init
    #-------------------------------------------------------------
    def stage1_process(self):
        pipelines_max = len2mask(self.n_pipelines)

        while True:
            pos = 0
            overlap = 0

            if (self.cores[self.pos].pipelines == pipelines_max):
                if (self.pos == 0):
                    return

                self.cores[self.pos].pipelines = 0
                self.pos -= 1
                continue
            #end if

            self.cores[self.pos].pipelines += 1
            
            if (popcount(self.cores[self.pos].pipelines) \
                            != self.cores[self.pos].n_pipelines):
                continue

            overlap = 0
            pos = 0
            while (pos < self.pos):
                if ((self.cores[self.pos].pipelines) & \
                        (self.cores[pos].pipelines)):
                    overlap = 1
                    break
                pos += 1
            #end while

            if (overlap):
                continue

            
            if ((self.pos > 0) and \
               ((self.cores[self.pos].n_pipelines) == (self.cores[self.pos - 1].n_pipelines)) and \
               ((self.cores[self.pos].pipelines) < (self.cores[self.pos - 1].pipelines))):
                continue

            if (self.pos == self.n_cores - 1):
                self.stage1_log()
                self.c2.stage2_init(self)
                self.c2.stage2_process()

                if (self.pos == 0):
                    return

                self.cores[self.pos].pipelines = 0
                self.pos -= 1
                continue
            #endif

            self.pos += 1
        #end for
    #end stage1_process
            
            
    #-------------------------------------------------------------
    def stage1_log(self):
        tmp_file_comment = ""
        if(enable_stage1_traceout == 1):
            print('STAGE1: ', end = '')
            tmp_file_comment += 'STAGE1: '
            i = 0
            while (i < self.n_cores):
                print('C%d = [' %i, end='')
                tmp_file_comment += "C{} = [".format(i)

                j = self.n_pipelines - 1
                while (j >= 0):
                    cond = ((self.cores[i].pipelines) & (1 << j))
                    if (cond):
                        print('1', end='')
                        tmp_file_comment += '1'
                    else:
                        print('0', end='')
                        tmp_file_comment += '0'
                    j -= 1
            
                print(']\t', end='')
                tmp_file_comment += ']\t'
                i += 1
            #end while
            print('\n', end ='')
            #tmp_file_comment += '\n'
            self.stage1_file_comment = tmp_file_comment
            self.c2.stage1_file_comment = tmp_file_comment
        #endif

        #check if file traceing is enabled         
        if(enable_stage1_fileout != 1):
            return

        #spit out the combination to file
        self.stage1_process_file()
    #end function stage1_log


    #------------------------------------------------------------------------ 
    def stage1_updateCoresInBuf(self, nPipeline, sCore):

        rePipeline = self._fileTrace.arr_pipelines[nPipeline]
        rePipeline = rePipeline.replace("[","\[").replace("]","\]")
        reCore = 'core\s*=\s*((\d*)|(((s|S)\d)?(c|C)[1-9][0-9]*)).*\n'
        sSubs = 'core = ' + sCore + '\n'

        reg_pipeline = re.compile(rePipeline)
        search_match = reg_pipeline.search(self._fileTrace.in_buf)
        #debug
        #print(search_match)

        if(search_match):
            pos = search_match.start()
            substr1 = self._fileTrace.in_buf[:pos]
            substr2 = self._fileTrace.in_buf[pos:]
            substr2 = re.sub(reCore, sSubs, substr2,1)
            self._fileTrace.in_buf = substr1 + substr2
        #endif
    #end function stage1_updateCoresInBuf

    #------------------------------------------------------------------------ 
    def stage1_process_file(self):
        outFileName = os.path.join(self._fileTrace.out_path, \
                self._fileTrace.prefix_outfile)
        
        i = 0 #represents core number
        while (i < self.n_cores):
            outFileName += '_'
            outFileName += str(self.cores[i].pipelines)

            j = self.n_pipelines - 1
            pipeline_idx = 0
            while(j >= 0):
                cond = ((self.cores[i].pipelines) & (1 << j))
                if (cond):
                    #update the pipelines array to match the core
                    # only in case of cond match
                    self.arr_pipelines2cores[pipeline_idx] = fileTrace.in_physical_cores[i]
                #endif
                j -= 1
                pipeline_idx += 1
            #end while
            i += 1
        #end while

        # update the in_buf as per the arr_pipelines2cores      
        for pipeline_idx in range(len(self.arr_pipelines2cores)):
            self.stage1_updateCoresInBuf(pipeline_idx,self.arr_pipelines2cores[pipeline_idx])

        
        #by now the in_buf is all set to be written to file 
        outFileName += self._fileTrace.suffix_outfile
        outputFile = open(outFileName, "w")
        
        #write out the comments 
        outputFile.write("; =============== Pipeline-to-Core Mapping ================\n")
        outputFile.write("; Generated from file {}\n".format(self._fileTrace.in_file_namepath))
        outputFile.write("; Input pipelines = {}\n; Input cores = {}\n" \
                .format(fileTrace.arr_pipelines, fileTrace.in_physical_cores))

        strTruncated = ("", "(Truncated)") [self._fileTrace.ncores_truncated]
        outputFile.write("; N_PIPELINES = {} N_CORES = {} {}\n"\
                .format(self._fileTrace.n_pipelines, self._fileTrace.n_cores, strTruncated))

        outputFile.write("; {}\n".format(self.stage0_file_comment)) #stage0 comment
        outputFile.write("; {}\n".format(self.stage1_file_comment)) #stage1 comment
        #debugging
        #outputFile.write("; <<<<printing stage1 arr_pipelines2cores>>>>")
        #outputFile.write("; stage1_arr_pipelines2cores = {}".format(self.arr_pipelines2cores))
        outputFile.write("; ========================================================\n")
        outputFile.write(";\n")

        #
        outputFile.write(self._fileTrace.in_buf)
        outputFile.flush()
        outputFile.close()
    #end function stage1_process_file
# end class Context1


#----------------------------------------------------------------------------- 
class Context2:
    #class attribute
    _fileTrace = None

    def __init__(self):
        self.cores = [Cores2() for i in range(constants.MAX_CORES)]
        self.n_cores = 0
        self.n_pipelines = 0
        self.pos = 0
        self.stage0_file_comment = ""
        self.stage1_file_comment = ""
        self.stage2_file_comment = ""

        #each array entry is a pipeline mapped to core stored as string
        # pipeline ranging from 1 to n, however stored in zero based array
        self.arr2_pipelines2cores = []

    #-------------------------------------------------------------
    def stage2_print(self):
        print('printing Context2 obj')
        print('c2.cores(pipelines, n_pipelines, counter, counter_max) =')
        for cores_count in range(0, constants.MAX_CORES):
            print('core[%d] = (%d,%d,%d,%d)' %(cores_count,
                                    self.cores[cores_count].pipelines, \
                                    self.cores[cores_count].n_pipelines, \
                                    self.cores[cores_count].counter, \
                                    self.cores[cores_count].counter_max))

            print('c2.n_cores = %d' %self.n_cores, end='')
            print('c2.n_pipelines = %d' %self.n_pipelines, end='')
            print('c2.pos = %d' %self.pos)
            print('c2.stage0_file_comment = %s' %self.self.stage0_file_comment)
            print('c2.stage1_file_comment = %s' %self.self.stage1_file_comment)
            print('c2.stage2_file_comment = %s' %self.self.stage2_file_comment)
        #end for
    #end function stage2_print
            

    #-------------------------------------------------------------
    def stage2_reset(self):
        for i in range(0, constants.MAX_CORES):
            self.cores[i].pipelines = 0
            self.cores[i].n_pipelines = 0;
            self.cores[i].counter = 0;
            self.cores[i].counter_max = 0
            
            for idx in range(0, constants.MAX_PIPELINES):
                self.cores[i].bitpos[idx] = 0

        self.n_cores = 0
        self.n_pipelines = 0
        self.pos = 0

        self.arr2_pipelines2cores.clear() 
    #end stage2_reset

    #-------------------------------------------------------------
    def bitpos_load(self, coreidx):
        i = j = 0
        while (i < self.n_pipelines):
            if ((self.cores[coreidx].pipelines) & \
                 (1 << i)):
                self.cores[coreidx].bitpos[j] = i
                j += 1
            i += 1
        self.cores[coreidx].n_pipelines = j


    #-------------------------------------------------------------
    def bitpos_apply(self, in_buf, pos, n_pos):
        out = 0
        for i in range(0, n_pos):
            out |= (in_buf & (1 << i)) << (pos[i] - i)

        return out


    #-------------------------------------------------------------
    def stage2_init(self, c1):
        self.stage2_reset()
        self.n_cores = c1.n_cores
        self.n_pipelines = c1.n_pipelines

        self.arr2_pipelines2cores = [''] * self.n_pipelines

        core_idx = 0
        while (core_idx < self.n_cores):
            self.cores[core_idx].pipelines = c1.cores[core_idx].pipelines

            self.bitpos_load(core_idx)
            core_idx += 1
        #end while 
    #end function stage2_init


    #-------------------------------------------------------------
    def stage2_log(self):
        tmp_file_comment = ""
        if(enable_stage2_traceout == 1):
            print('STAGE2: ', end='')
            tmp_file_comment += 'STAGE2: '

            for i in range(0, self.n_cores):
                mask = len2mask(self.cores[i].n_pipelines)
                pipelines_ht0 = self.bitpos_apply((~self.cores[i].counter) & mask, \
                                self.cores[i].bitpos, \
                                self.cores[i].n_pipelines)

                pipelines_ht1 = self.bitpos_apply(self.cores[i].counter, \
                                self.cores[i].bitpos, \
                                self.cores[i].n_pipelines)

                print('C%dHT0 = [' %i, end='')
                tmp_file_comment += "C{}HT0 = [".format(i)
                tmp_file_comment += bitstring_write(pipelines_ht0, self.n_pipelines)

                print(']\tC%dHT1 = [' %i, end='')
                tmp_file_comment += "]\tC{}HT1 = [".format(i)
                tmp_file_comment += bitstring_write(pipelines_ht1, self.n_pipelines)
                print(']\t', end='')
                tmp_file_comment += ']\t'
            #end for
            print('')
            self.stage2_file_comment = tmp_file_comment
        #endif

        #check if file traceing is enabled         
        if(enable_stage2_fileout != 1):
            return 
        #spit out the combination to file
        self.stage2_process_file()

    #end function stage2_log

    #-------------------------------------------------------------
    def stage2_updateCoresInBuf(self, nPipeline, sCore):
        rePipeline = self._fileTrace.arr_pipelines[nPipeline]
        rePipeline = rePipeline.replace("[","\[").replace("]","\]")
        reCore = 'core\s*=\s*((\d*)|(((s|S)\d)?(c|C)[1-9][0-9]*)).*\n'
        sSubs = 'core = ' + sCore + '\n'
        #sSubs = 'core = ' + self._fileTrace.in_physical_cores[sCore] + '\n'

        reg_pipeline = re.compile(rePipeline)
        search_match = reg_pipeline.search(self._fileTrace.in_buf)

        if(search_match):
            pos = search_match.start()
            substr1 = self._fileTrace.in_buf[:pos]
            substr2 = self._fileTrace.in_buf[pos:]
            substr2 = re.sub(reCore, sSubs, substr2,1)
            self._fileTrace.in_buf = substr1 + substr2
        #endif
    #end function stage2_updateCoresInBuf


    #-------------------------------------------------------------
    def pipelines2cores(self, n, n_bits, nCore, bHT):
        if (n_bits > 64):
            return

        i = n_bits - 1
        pipeline_idx = 0
        while (i >= 0):
            cond = (n & (1 << i))
            if (cond):
                #update the pipelines array to match the core
                # only in case of cond match
                # PIPELINE0 and core 0 are reserved
                if(bHT):
                    tmpCore = fileTrace.in_physical_cores[nCore] + 'h'
                    self.arr2_pipelines2cores[pipeline_idx] = tmpCore
                else:
                    self.arr2_pipelines2cores[pipeline_idx] = \
                            fileTrace.in_physical_cores[nCore]
                #endif
            #endif
            i -= 1
            pipeline_idx += 1
        #end while

    #end function pipelines2cores

    #-------------------------------------------------------------
    def stage2_process_file(self):
        outFileName = os.path.join(self._fileTrace.out_path, \
                self._fileTrace.prefix_outfile)

        for i in range(0, self.n_cores):
            mask = len2mask(self.cores[i].n_pipelines)
            pipelines_ht0 = self.bitpos_apply((~self.cores[i].counter) & mask, \
                                self.cores[i].bitpos, \
                                self.cores[i].n_pipelines)

            pipelines_ht1 = self.bitpos_apply(self.cores[i].counter, \
                                self.cores[i].bitpos, \
                                self.cores[i].n_pipelines)
   
            outFileName += '_'
            outFileName += str(pipelines_ht0)
            outFileName += '_'
            outFileName += str(pipelines_ht1)
            outFileName += 'HT'            

            #update pipelines to core mapping
            self.pipelines2cores(pipelines_ht0, self.n_pipelines, i , False)
            self.pipelines2cores(pipelines_ht1, self.n_pipelines, i, True)
        #end for

        # update the in_buf as per the arr_pipelines2cores 
        for pipeline_idx in range(len(self.arr2_pipelines2cores)):
            self.stage2_updateCoresInBuf(pipeline_idx, self.arr2_pipelines2cores[pipeline_idx])
       
        #by now the in_buf is all set to be written to file 
        outFileName += self._fileTrace.suffix_outfile
        outputFile = open(outFileName, "w")

        #write out the comments
        outputFile.write("; ========= Pipeline-to-Core Mapping =====================\n")
        outputFile.write("; Generated from file {}\n".format(self._fileTrace.in_file_namepath))
        outputFile.write("; Input pipelines = {}\n; Input cores = {}\n" \
                .format(fileTrace.arr_pipelines, fileTrace.in_physical_cores))

        strTruncated = ("", "(Truncated)") [self._fileTrace.ncores_truncated]
        outputFile.write("; N_PIPELINES = {} N_CORES = {} {}\n"\
                .format(self._fileTrace.n_pipelines, self._fileTrace.n_cores, strTruncated))

        outputFile.write("; {}\n".format(self.stage0_file_comment)) #stage0 comment
        outputFile.write("; {}\n".format(self.stage1_file_comment)) #stage1 comment
        outputFile.write("; {}\n".format(self.stage2_file_comment)) #stage2 comment
        outputFile.write("; ========================================================\n")
        outputFile.write(";\n")

        outputFile.write(self._fileTrace.in_buf)
        outputFile.flush()
        outputFile.close()

    #end function stage2_process_file 

    #-------------------------------------------------------------
    def stage2_process(self):
        i = 0
        while(i < self.n_cores):
            self.cores[i].counter_max = len2mask(self.cores[i].n_pipelines - 1)
            i += 1
        #end while
        
        self.pos = self.n_cores - 1
        while True:
            if (self.pos == self.n_cores - 1):
                self.stage2_log()

            if (self.cores[self.pos].counter == self.cores[self.pos].counter_max):
                if (self.pos == 0):
                    return
                #endif

                self.cores[self.pos].counter = 0
                self.pos -= 1
                continue
            #endif

            self.cores[self.pos].counter += 1

            if(self.pos < self.n_cores - 1):
                self.pos += 1
            #endif
        #end while
#end class Context2

#----------------------------------------------------------------------------- 
class FileTrace:
    #initialize default parameters
    def __init__(self,filenamepath):
        self.in_file_namepath = os.path.abspath(filenamepath)
        self.in_filename = os.path.basename(self.in_file_namepath) 
        self.in_path = os.path.dirname(self.in_file_namepath)
        
        #set output folder same as input
        self.out_path = self.in_path

        filenamesplit = self.in_filename.split('.')
        self.prefix_outfile = filenamesplit[0]
        self.suffix_outfile = ".cfg"
        self.in_buf = None
        self.arr_pipelines = []  # holds the positions of search
        
        self.max_cores = 15 
        self.max_pipelines = 15

        self.in_physical_cores = None
        self.hyper_thread = None

        # save the num of pipelines determined from input file
        self.n_pipelines = 0
        # save the num of cores input (or the truncated value) 
        self.n_cores = 0
        self.ncores_truncated = False
       
    #end init

    def print_TraceFile(self):
        print("self.in_file_namepath = ", self.in_file_namepath) 
        print("self.in_filename = ", self.in_filename)
        print("self.in_path = ", self.in_path)
        print("self.out_path = ", self.out_path)
        print("self.prefix_outfile = ", self.prefix_outfile)
        print("self.suffix_outfile = ", self.suffix_outfile)
        print("self.in_buf = ", self.in_buf)
        print("self.arr_pipelines =", self.arr_pipelines)
        print("self.in_physical_cores", self.in_physical_cores)
        print("self.hyper_thread", self.hyper_thread)
    #end function print_TraceFile

#end class FileTrace


#-----------------------------------------------------------------------------
# main process method
#
def process(n_cores, n_pipelines, fileTrace):
    if (n_cores == 0):
        sys.exit('N_CORES is 0, exiting')
    #endif

    if (n_pipelines == 0):
        sys.exit('N_PIPELINES is 0, exiting')
    #endif

    if (n_cores > n_pipelines):
        print('\nToo many cores, truncating N_CORES to N_PIPELINES')
        n_cores = n_pipelines
        fileTrace.ncores_truncated = True
    #endif
    fileTrace.n_pipelines = n_pipelines
    fileTrace.n_cores = n_cores

    strTruncated = ("", "(Truncated)") [fileTrace.ncores_truncated]
    print("N_PIPELINES = {}, N_CORES = {} {}" \
            .format(n_pipelines,n_cores, strTruncated))
    print("---------------------------------------------------------------")

    c0 = Context0()
    c1 = Context1()
    c2 = Context2()

    #initialize the class variables
    c1._fileTrace = fileTrace
    c2._fileTrace = fileTrace

    c0.stage0_init(n_cores, n_pipelines, c1, c2)
    c0.stage0_process()

#end function process



#--------------------------------------------------------------------------
def validate_core(core):
	match = reg_phycore.match(core)
	if(match):
		return True
	else:
		return False
        #endif
#end function validate_core


#--------------------------------------------------------------------------
def validate_phycores(phy_cores):
        #eat up whitespaces
        phy_cores = phy_cores.strip().split(',')

        #check if the core list is unique
        if(len(phy_cores) != len(set(phy_cores))):
                print('list of physical cores has duplicates')
                return False
        #endif

        for core in phy_cores:
                if(validate_core(core) != True):
                    print('invalid physical core specified.')
                    return None
                else:
                    return phy_cores
                #endif
        #endfor
#end function validate_phycores

#--------------------------------------------------------------------------
def scanconfigfile(fileTrace):
    #debug
    #fileTrace.print_TraceFile()

    # open file
    filetoscan = open(fileTrace.in_file_namepath, 'r')
    fileTrace.in_buf = filetoscan.read()

    #reset iterator on open file
    filetoscan.seek(0)

    # scan input file for pipelines
    # master pipelines to be ignored
    pattern_pipeline = r'\[PIPELINE\d*\]'
    pattern_mastertype = r'type\s*=\s*MASTER'

    pending_pipeline = False
    for line in filetoscan:
        match_pipeline = re.search(pattern_pipeline, line)
        match_type = re.search('type\s*=', line)
        match_mastertype = re.search(pattern_mastertype, line)

        if(match_pipeline):
            sPipeline = line[match_pipeline.start():match_pipeline.end()]
            #sPipeline = sPipeline.strip('[]')
            pending_pipeline = True
        elif(match_type):
            # found a type definition...
            if(match_mastertype == None):
                # and this is not a master pipeline...
                if(pending_pipeline == True):
                    # add it to the list of pipelines to be mapped
                    fileTrace.arr_pipelines.append(sPipeline)
                    pending_pipeline = False
            else:
                # and this is a master pipeline...
                # ignore the current and move on to next
                sPipeline = ""
                pending_pipeline = False
            #endif
        #endif
    #endfor
    filetoscan.close()

    # validate if pipelines are unique
    if(len(fileTrace.arr_pipelines) != len(set(fileTrace.arr_pipelines))):
        sys.exit('Error: duplicate pipelines in input file')
    #endif

    num_pipelines = len(fileTrace.arr_pipelines)
    num_cores = len(fileTrace.in_physical_cores)

    #debug
    #print(fileTrace.matches_pipeline)
    print("-------------------Pipeline-to-core mapping--------------------")
    print("Input pipelines = {}\nInput cores = {}" \
            .format(fileTrace.arr_pipelines, fileTrace.in_physical_cores))

    #input configuration file validations goes here
    if (num_cores > fileTrace.max_cores):
        sys.exit('Error: number of cores specified > max_cores (%d)' %fileTrace.max_cores)

    if (num_pipelines > fileTrace.max_pipelines):
        sys.exit('Error: number of pipelines in input cfg file > max_pipelines (%d)' %fileTrace.max_pipelines)

    #call process to generate pipeline-to-core mapping, trace and log
    process(num_cores, num_pipelines, fileTrace)

#end function scanconfigfile


#-----------------------------------------------------------------------------
# python trick - so that our Python files can act as either reusable modules, 
# or as standalone programs
if __name__ == "__main__":

    parser = argparse.ArgumentParser(description='mappipelines')

    reqNamedGrp = parser.add_argument_group('required named args')
    reqNamedGrp.add_argument('-i', '--input-file', type=argparse.FileType('r'), help='Input config file', required=True)

    #--physical-cores “<core>, <core>, …”, with <core> = s<SOCKETID>c<COREID>
    reqNamedGrp.add_argument('-pc', '--physical-cores', type=validate_phycores, help='''Enter available CPU cores in format:\"<core>,<core>,...\"
        where each core format: \"s<SOCKETID>c<COREID>\"
        where SOCKETID={0..9}, COREID={1-99}''', required=True)

    #add optional arguments
    parser.add_argument('-ht', '--hyper-thread', help='enable/disable hyper threading. default is ON', default='ON', choices=['ON','OFF'])

    parser.add_argument('-nO', '--no-output-file', help='disable output config file generation. Output file generation is enabled by default', action="store_true")

    args = parser.parse_args()

    # create object of FileTrace and initialise
    fileTrace = FileTrace(args.input_file.name)
    fileTrace.in_physical_cores = args.physical_cores
    fileTrace.hyper_thread = args.hyper_thread

    if(fileTrace.hyper_thread == 'OFF'):
        print("!!!!disabling stage2 HT!!!!")
        enable_stage2_traceout = 0
        enable_stage2_fileout = 0
    #endif

    if(args.no_output_file == True):
        print("!!!!disabling stage1 and stage2 fileout!!!!")
        enable_stage1_fileout = 0
        enable_stage2_fileout = 0
    #endif

    scanconfigfile(fileTrace)

#end main
