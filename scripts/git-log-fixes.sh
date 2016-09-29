#! /bin/sh -e

if [ $# -lt 1 ] ; then
	echo 'range argument required' >&2
	exit 1
fi
range="$*"

# get major release version of a commit
commit_version () # <hash>
{
	tag=$(git tag -l --contains $1 | head -n1)
	if [ -z "$tag" ] ; then
		# before -rc1 tag of release in progress
		make showversion | cut -d'.' -f-2
	else
		echo $tag | sed 's,^v,,' | sed 's,-rc.*,,'
	fi
}

# get bug origin hashes of a fix
origin_filter () # <hash>
{
	git log --format='%b' -1 $1 |
	sed -n 's,^ *\([Ff]ixes\|[Rr]everts\): *\([0-9a-f]*\).*,\2,p'
}

# get oldest major release version of bug origins
origin_version () # <origin_hash> ...
{
	for origin in $* ; do
		# check hash is valid
		git rev-parse -q --verify $1 >&- || continue
		# get version of this bug origin
		local origver=$(commit_version $origin)
		local roothashes=$(origin_filter $origin)
		if [ -n "$roothashes" ] ; then
			# look chained fix of fix recursively
			local rootver="$(origin_version $roothashes)"
			[ -n "$rootver" ] || continue
			echo "$rootver (partially fixed in $origver)"
		else
			echo "$origver"
		fi
	# filter the oldest origin
	done | sort -uV | head -n1
}

git log --oneline --reverse $range |
while read id headline ; do
	origins=$(origin_filter $id)
	[ -n "$origins" ] || echo "$headline" | grep -q fix || continue
	version=$(commit_version $id)
	if [ -n "$origins" ] ; then
		origver="$(origin_version $origins)"
		[ -n "$origver" ] || continue
		# ignore fix of bug introduced in the same release
		! echo "$origver" | grep -q "^$version" || continue
	else
		origver='N/A'
	fi
	printf '%s %7s %s (%s)\n' $version $id "$headline" "$origver"
done
