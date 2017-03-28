# Copyright 2014 6WIND S.A.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# - Redistributions of source code must retain the above copyright
#   notice, this list of conditions and the following disclaimer.
#
# - Redistributions in binary form must reproduce the above copyright
#   notice, this list of conditions and the following disclaimer in
#   the documentation and/or other materials provided with the
#   distribution.
#
# - Neither the name of 6WIND S.A. nor the names of its
#   contributors may be used to endorse or promote products derived
#   from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
# HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
# STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
# OF THE POSSIBILITY OF SUCH DAMAGE.

Name: dpdk
Version: 17.02
Release: 1
Packager: packaging@6wind.com
URL: http://dpdk.org
Source: http://dpdk.org/browse/dpdk/snapshot/dpdk-%{version}.tar.gz

Summary: Data Plane Development Kit core
Group: System Environment/Libraries
License: BSD and LGPLv2 and GPLv2

%define with_dkms %{?_without_dkms: 0} %{?!_without_dkms: 1}

ExclusiveArch: i686 x86_64 aarch64
%ifarch aarch64
%global machine armv8a
%global target arm64-%{machine}-linuxapp-gcc
%global config arm64-%{machine}-linuxapp-gcc
%else
%global machine default
%global target %{_arch}-%{machine}-linuxapp-gcc
%global config %{_arch}-native-linuxapp-gcc
%endif

BuildRequires: kernel-devel, kernel-headers, libpcap-devel
%ifarch i686 x86_64
BuildRequires: xen-devel
%endif
BuildRequires: doxygen, python-sphinx, inkscape
BuildRequires: texlive-collection-latexextra

%description
DPDK core includes kernel modules, core libraries and tools.
testpmd application allows to test fast packet processing environments
on x86 platforms. For instance, it can be used to check that environment
can support fast path applications such as 6WINDGate, pktgen, rumptcpip, etc.
More libraries are available as extensions in other packages.

%package devel
Summary: Data Plane Development Kit for development
Requires: %{name}%{?_isa} = %{version}-%{release}
%description devel
DPDK devel is a set of makefiles, headers and examples
for fast packet processing on x86 platforms.

%if %{with_dkms}
%package igb-uio
Summary: Data Plane Development Kit, igb_uio kernel module
Group: System/Kernel
Requires: %{name}%{?_isa} = %{version}-%{release}
Requires: gcc, make
Requires(post):   dkms
Requires(preun):  dkms
%description igb-uio
Data Plane Development Kit, igb_uio kernel module

%package rte-kni
Summary: Data Plane Development Kit, rte_kni kernel module
Group: System/Kernel
Requires: %{name}%{?_isa} = %{version}-%{release}
Requires: gcc, make
Requires(post):   dkms
Requires(preun):  dkms
%description rte-kni
Data Plane Development Kit, rte_kni kernel module
%endif

%package doc
Summary: Data Plane Development Kit API documentation
BuildArch: noarch
%description doc
DPDK doc is divided in two parts: API details in doxygen HTML format
and guides in sphinx HTML/PDF formats.

%prep
%setup -q

%build
make O=%{target} T=%{config} config
sed -ri 's,(RTE_MACHINE=).*,\1%{machine},' %{target}/.config
sed -ri 's,(RTE_APP_TEST=).*,\1n,'         %{target}/.config
%if %{with_dkms}
sed -ri 's,(CONFIG_RTE_EAL_IGB_UIO=).*,\1n,' %{target}/.config
sed -ri 's,(CONFIG_RTE_KNI_KMOD=).*,\1n,'    %{target}/.config
%endif
sed -ri 's,(RTE_BUILD_SHARED_LIB=).*,\1y,' %{target}/.config
sed -ri 's,(RTE_NEXT_ABI=).*,\1n,'         %{target}/.config
sed -ri 's,(LIBRTE_VHOST=).*,\1y,'         %{target}/.config
sed -ri 's,(LIBRTE_PMD_PCAP=).*,\1y,'      %{target}/.config
%ifarch i686 x86_64
sed -ri 's,(LIBRTE_PMD_XENVIRT=).*,\1y,'   %{target}/.config
%endif
make O=%{target} %{?_smp_mflags}
make O=%{target} doc

%install
rm -rf %{buildroot}
make install O=%{target} DESTDIR=%{buildroot} \
	prefix=%{_prefix} bindir=%{_bindir} sbindir=%{_sbindir} \
	includedir=%{_includedir}/dpdk libdir=%{_libdir} \
	datadir=%{_datadir}/dpdk docdir=%{_docdir}/dpdk

%if %{with_dkms}
# Kernel module sources install for dkms
%{__mkdir_p} %{buildroot}%{_usrsrc}/dpdk-igb-uio-%{version}/
%{__cp} -r lib/librte_eal/linuxapp/igb_uio/* %{buildroot}%{_usrsrc}/dpdk-igb-uio-%{version}/

%{__mkdir_p} %{buildroot}%{_usrsrc}/dpdk-rte-kni-%{version}/
%{__cp} -r lib/librte_eal/linuxapp/kni/* %{buildroot}%{_usrsrc}/dpdk-rte-kni-%{version}/

cat > %{buildroot}%{_datadir}/dpdk/buildtools/dpdk-sdk-env.sh << EOF
export RTE_TARGET=%{target}
export RTE_SDK="/usr/share/dpdk/"
export RTE_INCLUDE="/usr/include/dpdk"
EOF

# Prepare dkms.conf
cat > %{buildroot}%{_usrsrc}/dpdk-igb-uio-%{version}/dkms.conf << EOF

PACKAGE_NAME="dpdk-igb-uio"
PACKAGE_VERSION="%{version}-%{release}"
MAKE="source /usr/share/dpdk/buildtools/dpdk-sdk-env.sh; make MODULE_CFLAGS='-I/usr/include/dpdk -include /usr/include/dpdk/rte_config.h'"
CLEAN="source /usr/share/dpdk/buildtools/dpdk-sdk-env.sh; make clean"
BUILT_MODULE_NAME[0]=igb_uio
DEST_MODULE_LOCATION[0]=/updates/dkms
AUTOINSTALL=yes
EOF

# Prepare dkms.conf
cat > %{buildroot}%{_usrsrc}/dpdk-rte-kni-%{version}/dkms.conf << EOF
PACKAGE_NAME="dpdk-rte-kni"
PACKAGE_VERSION="%{version}-%{release}"
MAKE="source /usr/share/dpdk/buildtools/dpdk-sdk-env.sh; make MODULE_CFLAGS='-I/usr/include/dpdk -include /usr/include/dpdk/rte_config.h -I%{_usrsrc}/dpdk-rte-kni-%{version}/ethtool/ixgbe -I%{_usrsrc}/dpdk-rte-kni-%{version}/ethtool/igb'"
CLEAN="source /usr/share/dpdk/buildtools/dpdk-sdk-env.sh; make clean"
BUILT_MODULE_NAME[0]=rte_kni
DEST_MODULE_LOCATION[0]=/updates/dkms
AUTOINSTALL="YES"
EOF
%endif

%files
%dir %{_datadir}/dpdk
%{_datadir}/dpdk/usertools
%if ! %{with_dkms}
/lib/modules/%(uname -r)/extra/*
%endif
%{_sbindir}/*
%{_bindir}/*
%{_libdir}/*

%files devel
%{_includedir}/dpdk
%{_datadir}/dpdk/mk
%{_datadir}/dpdk/buildtools
%{_datadir}/dpdk/%{target}
%{_datadir}/dpdk/examples

%if %{with_dkms}
%files igb-uio
%defattr(-,root,root)
%{_usrsrc}/dpdk-igb-uio-%{version}/

%files rte-kni
%defattr(-,root,root)
%{_usrsrc}/dpdk-rte-kni-%{version}/
%endif

%files doc
%doc %{_docdir}/dpdk

%post
/sbin/ldconfig
/sbin/depmod

%if %{with_dkms}
%post igb-uio
# Add to DKMS registry
isadded=`dkms status -m "dpdk-igb-uio" -v "%{version}"`
if [ "x${isadded}" = "x" ] ; then
    dkms add -m "dpdk-igb-uio" -v "%{version}" || :
fi
dkms build -m "dpdk-igb-uio" -v "%{version}" || :
dkms install -m "dpdk-igb-uio" -v "%{version}" --force || :

%post rte-kni
# Add to DKMS registry
isadded=`dkms status -m "dpdk-rte-kni" -v "%{version}"`
if [ "x${isadded}" = "x" ] ; then
    dkms add -m "dpdk-rte-kni" -v "%{version}" || :
fi
dkms build -m "dpdk-rte-kni" -v "%{version}" || :
dkms install -m "dpdk-rte-kni" -v "%{version}" --force || :

%preun igb-uio
dkms remove -m "dpdk-igb-uio" -v "%{version}" --all || :

%preun rte-kni
dkms remove -m "dpdk-rte-kni" -v "%{version}" --all || :
%endif

%postun
/sbin/ldconfig
/sbin/depmod
