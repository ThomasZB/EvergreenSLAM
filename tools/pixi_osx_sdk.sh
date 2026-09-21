# Sourced by pixi on activation (macOS only). Pins SDKROOT to a macOS SDK the environment's
# linker can actually read.
#
# The macOS 27 SDK's .tbd stubs list an `arm64e.x1-*` target that conda-forge's ld64 956.6
# rejects as malformed. It then drops libSystem entirely and every libc symbol comes back
# undefined at link time, which reads like a broken build rather than a toolchain mismatch.
# Pick the newest installed SDK without that token instead; drop this file once conda-forge
# ships an ld64 that parses it. An SDKROOT already set by the caller always wins.

if [ -z "${SDKROOT:-}" ]; then
  _evergreen_sdk=$(
    for _d in /Library/Developer/CommandLineTools/SDKs/MacOSX*.sdk \
              /Applications/Xcode.app/Contents/Developer/Platforms/MacOSX.platform/Developer/SDKs/MacOSX*.sdk; do
      [ -r "$_d/usr/lib/libSystem.tbd" ] || continue
      grep -q 'arm64e\.x1' "$_d/usr/lib/libSystem.tbd" && continue
      _v=$(basename "$_d" .sdk)
      _v=${_v#MacOSX}
      printf '%s\t%s\n' "${_v:-0}" "$_d"
    done | sort -V | tail -1 | cut -f2
  )
  if [ -n "$_evergreen_sdk" ]; then
    export SDKROOT="$_evergreen_sdk"
    export CONDA_BUILD_SYSROOT="$_evergreen_sdk"
  else
    echo "pixi_osx_sdk.sh: no macOS SDK without arm64e.x1 found; ld64 will fail to link" >&2
  fi
  unset _evergreen_sdk
fi
