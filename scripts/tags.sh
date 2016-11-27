#!/bin/bash
# Generate tags or gtags or cscope files
# Usage tags.sh <mode> T=<target> [VERBOSE=1]
# set -x

verbose=false
linuxapp=false
bsdapp=false
x86_64=false
arm=false
arm64=false
ia_32=false
ppc_64=false
tile=false

if [ "$VERBOSE" = "1" ]; then
	verbose=true
fi

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

skiplist="${skip_bsd} ${skip_linux} ${skip_arch} ${skip_sse} ${skip_avx} \
		 ${skip_neon} ${skip_altivec} ${skip_x86} ${skip_arm64}"

find_sources()
{
	find $1 $ignore $3 -name $2 -not -type l -print
}

common_sources()
{
	find_sources "${source_dirs}" '*.[chS]' "$skiplist"
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
	find_sources "${source_dirs}" '*neon*.[chS]'
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
	find_sources "${source_dirs}" '*arm64.[chS]'
}

ia_common()
{
	find_sources "lib/librte_eal/common/arch/x86" '*.[chS]'

	find_sources "examples/performance-thread/common/arch/x86" '*.[chS]'
	find_sources "${source_dirs}" '*_sse*.[chS]'
	find_sources "${source_dirs}" '*_avx*.[chS]'
	find_sources "${source_dirs}" '*x86.[chS]'
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
	find_sources "${source_dirs}" '*altivec*.[chS]'
}

tile_sources()
{
	find_sources "lib/librte_eal/common/arch/tile" '*.[chS]'
	find_sources "lib/librte_eal/common/include/arch/tile" '*.[chS]'
}

config_file()
{
	if [ -f $RTE_OUTPUT/include/rte_config.h ]; then
		ls $RTE_OUTPUT/include/rte_config.h
	fi
}

usage()
{
	if [ "$1" = 'tags' ] ; then
		echo "Generate tags file for editors"
	fi
	if [ "$1" = 'cscope' ] ; then
		echo "Generate cscope index"
	fi
	if [ "$1" = 'gtags' ] ; then
		echo "Generate GNU GLOBAL index"
	fi
	echo "Usage: make tags|cscope|gtags T=<target> [VERBOSE=1]"
	echo "Valid targets are:"
	make showconfigs | sed 's,^,\t,'
}

check_valid_config()
{
	cfgfound=false
	allconfigs=`make showconfigs`
	for cfg in $allconfigs
	do
		if [ "$cfg" = "$1" ] ; then
			cfgfound=true
		fi
	done
	$cfgfound || echo "Invalid config: $1"
	$cfgfound || usage $2
	$cfgfound || exit 0
}

if [ -n "$T" ]; then
	check_valid_config $T $1

	if [ `echo $T | grep -c "linuxapp-" ` -gt 0 ]; then
		linuxapp=true
	fi

	if [ `echo $T | grep -c "bsdapp-" ` -gt 0 ]; then
		bsdapp=true
	fi

	if [ `echo $T | grep -c "x86_64-" ` -gt 0  ]; then
		x86_64=true
	fi

	if [ `echo $T | grep -c "i686-" ` -gt 0 ]; then
		ia_32=true
	fi

	if [ `echo $T | grep -c "x32-" ` -gt 0 ]; then
		ia_32=true
	fi

	if [ `echo $T | grep -c "arm-" ` -gt 0 ]; then
		arm=true
	fi

	if [ `echo $T | grep -c "arm64-" ` -gt 0 ]; then
		arm64=true
	fi

	if [ `echo $T | grep -c "ppc_64-" ` -gt 0 ]; then
		ppc_64=true
	fi

	if [ `echo $T | grep -c "tile-" ` -gt 0 ]; then
		tile=true
	fi

	$verbose && echo "linuxapp: $linuxapp"
	$verbose && echo "bsdapp:   $bsdapp"
	$verbose && echo "ia_32:    $ia_32"
	$verbose && echo "x86_64:   $x86_64"
	$verbose && echo "arm:      $arm"
	$verbose && echo "arm64:    $arm64"
	$verbose && echo "ppc_64:   $ppc_64"
	$verbose && echo "tile:     $tile"
	$verbose && echo "build:    $RTE_OUTPUT"
else
	usage $1
	exit
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
	$tile && tile_sources
	config_file
}

docscope()
{
	all_sources > cscope.files
	cscope -q -b -f cscope.out
}

dogtags()
{
	all_sources | gtags -i -f -
}

doctags()
{
	all_sources | xargs ctags -a
}

case "$1" in
	"cscope")
		docscope
		;;

	"gtags")
		dogtags
		;;
	"tags")
		rm -f tags
		doctags
		;;
esac
