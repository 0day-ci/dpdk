#!/bin/sh -e
# Generate tags or gtags or cscope or etags files
#
#   BSD LICENSE
#
#   Copyright 2017 Cavium Networks
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
#     * Neither the name of Cavium networks nor the names of its
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

verbose=false
linuxapp=false
bsdapp=false
x86_64=false
arm=false
arm64=false
ia_32=false
ppc_64=false

print_usage()
{
	echo "Usage: $(basename $0) [-h] [-v] tags|cscope|gtags|etags [config]"
	echo "Valid configs are:"
	make showconfigs | sed 's,^,\t,'
}

while getopts hv ARG ; do
	case $ARG in
		v ) verbose=true ;;
		h ) print_usage; exit 0 ;;
		? ) print_usage; exit 1 ;;
	esac
done
shift $(($OPTIND - 1))

#ignore version control files
ignore="( -name .svn -o -name CVS -o -name .hg -o -name .git ) -prune -o"

source_dirs="app buildtools drivers examples lib"

skip_bsd="( -name bsdapp ) -prune -o"
skip_linux="( -name linuxapp ) -prune -o"
skip_arch="( -name arch ) -prune -o"
skip_sse="( -name *_sse*.[chS] ) -prune -o"
skip_avx="( -name *_avx*.[chS] ) -prune -o"
skip_neon="( -name *_neon*.[chS] ) -prune -o"
skip_altivec="( -name *_altivec*.[chS] ) -prune -o"
skip_arm64="( -name *arm64*.[chS] ) -prune -o"
skip_x86="( -name *x86*.[chS] ) -prune -o"
skip_32b_files="( -name *_32.h ) -prune -o"
skip_64b_files="( -name *_64.h ) -prune -o"

skiplist="$skip_bsd $skip_linux $skip_arch $skip_sse $skip_avx \
		 $skip_neon $skip_altivec $skip_x86 $skip_arm64"

find_sources()
{
	find $1 $ignore $3 -name $2 -not -type l -print
}

common_sources()
{
	find_sources "$source_dirs" '*.[chS]' "$skiplist"
}

linuxapp_sources()
{
	find_sources "lib/librte_eal/linuxapp" '*.[chS]'
}

bsdapp_sources()
{
	find_sources "lib/librte_eal/bsdapp" '*.[chS]'
}

arm_common()
{
	find_sources "lib/librte_eal/common/arch/arm" '*.[chS]'
	find_sources "$source_dirs" '*neon*.[chS]'
}

arm_sources()
{
	arm_common
	find_sources "lib/librte_eal/common/include/arch/arm" '*.[chS]' \
					"$skip_64b_files"
}

arm64_sources()
{
	arm_common
	find_sources "lib/librte_eal/common/include/arch/arm" '*.[chS]' \
					 "$skip_32b_files"
	find_sources "$source_dirs" '*arm64.[chS]'
}

ia_common()
{
	find_sources "lib/librte_eal/common/arch/x86" '*.[chS]'

	find_sources "examples/performance-thread/common/arch/x86" '*.[chS]'
	find_sources "$source_dirs" '*_sse*.[chS]'
	find_sources "$source_dirs" '*_avx*.[chS]'
	find_sources "$source_dirs" '*x86.[chS]'
}

i686_sources()
{
	ia_common
	find_sources "lib/librte_eal/common/include/arch/x86" '*.[chS]' \
					"$skip_64b_files"
}

x86_64_sources()
{
	ia_common
	find_sources "lib/librte_eal/common/include/arch/x86" '*.[chS]' \
					"$skip_32b_files"
}

ppc64_sources()
{
	find_sources "lib/librte_eal/common/arch/ppc_64" '*.[chS]'
	find_sources "lib/librte_eal/common/include/arch/ppc_64" '*.[chS]'
	find_sources "$source_dirs" '*altivec*.[chS]'
}

config_file()
{
	if [ -f $RTE_SDK/$RTE_TARGET/include/rte_config.h ]; then
		ls $RTE_SDK/$RTE_TARGET/include/rte_config.h
	fi
}

check_valid_config()
{
	cfgfound=false
	allconfigs=$(make showconfigs)
	for cfg in $allconfigs
	do
		if [ "$cfg" = "$1" ] ; then
			cfgfound=true
		fi
	done
	$cfgfound || echo "Invalid config: $1"
	$cfgfound || print_usage
	$cfgfound || exit 0
}

if [ -n "$2" ]; then
	check_valid_config $2

	if [ $(echo $2 | grep -c "linuxapp-") -gt 0 ]; then
		linuxapp=true
	fi

	if [ $(echo $2 | grep -c "bsdapp-") -gt 0 ]; then
		bsdapp=true
	fi

	if [ $(echo $2 | grep -c "x86_64-") -gt 0  ]; then
		x86_64=true
	fi

	if [ $(echo $2 | grep -c "i686-") -gt 0 ]; then
		ia_32=true
	fi

	if [ $(echo $2 | grep -c "x32-") -gt 0 ]; then
		ia_32=true
	fi

	if [ $(echo $2 | grep -c "arm-") -gt 0 ]; then
		arm=true
	fi

	if [ $(echo $2 | grep -c "arm64-") -gt 0 ]; then
		arm64=true
	fi

	if [ $(echo $2 | grep -c "ppc_64-") -gt 0 ]; then
		ppc_64=true
	fi

else
	linuxapp=true
	bsdapp=true
	x86_64=true
	arm=true
	arm64=true
	ia_32=true
	ppc_64=true
fi

all_sources()
{
	common_sources
	$linuxapp && linuxapp_sources
	$bsdapp && bsdapp_sources
	$x86_64 && x86_64_sources
	$ia_32 && i686_sources
	$arm && arm_sources
	$arm64 && arm64_sources
	$ppc_64 && ppc64_sources
	config_file
}

show_flags()
{
	$verbose && echo "mode:     $1"
	$verbose && echo "config:   $2"
	$verbose && echo "linuxapp: $linuxapp"
	$verbose && echo "bsdapp:   $bsdapp"
	$verbose && echo "ia_32:    $ia_32"
	$verbose && echo "x86_64:   $x86_64"
	$verbose && echo "arm:      $arm"
	$verbose && echo "arm64:    $arm64"
	$verbose && echo "ppc_64:   $ppc_64"
	$verbose && echo "target:   $RTE_SDK/$RTE_TARGET"
}

case "$1" in
	"cscope")
		show_flags $1 $2
		all_sources > cscope.files
		cscope -q -b -f cscope.out
		;;
	"gtags")
		show_flags $1 $2
		all_sources | gtags -i -f -
		;;
	"tags")
		show_flags $1 $2
		rm -f tags
		all_sources | xargs ctags -a
		;;
	"etags")
		show_flags $1 $2
		rm -f TAGS
		all_sources | xargs etags -a
		;;
	*)
		echo "Invalid mode: $1"
		print_usage
		;;
esac
