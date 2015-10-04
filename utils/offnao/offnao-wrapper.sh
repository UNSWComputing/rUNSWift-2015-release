#!/bin/bash
CTC_DIR=@CTC_DIR@
if [[ -n "$CTC_DIR" ]]; then
  export QT_PLUGIN_PATH=$CTC_DIR/../sysroot_legacy/usr/lib/qt4/plugins/
  # Add boost_libs from Luke so we don't need to compile it as i386
  export LD_LIBRARY_PATH="$CTC_DIR/../sysroot_legacy/usr/lib/qt4/;$CTC_DIR/../boost_libs"
fi
`dirname $0`/offnao.bin "$@"
