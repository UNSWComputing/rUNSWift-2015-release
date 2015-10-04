#!/bin/bash

set -e
set -u

function myecho() {
  echo -n " [+] "
  echo -ne '\033[32;1m'
  echo -n $@
  echo -e '\033[0m'
}

OLD_RCD="${RUNSWIFT_CHECKOUT_DIR-}"
unset RUNSWIFT_CHECKOUT_DIR
unset CTC_DIR

# Set up git
cat << USER_CONFIG
If the user info is incorrect, please configure it like:
  git config user.name Jayen
  git config user.email jayen@cse.unsw.edu.au
USER_CONFIG
echo Your user name: $(git config user.name)
echo Your email: $(git config user.email)

# Set up ssh_config

for host in robot1 robot2
do
  if [ ! -f ~/.ssh/config ] || ! grep -q "Host $host" ~/.ssh/config ; then (
      echo "Host $host"
      echo "  Hostname $host.local"
      echo "  HostKeyAlias $host"
      echo "  CheckHostIP no"
      echo "  User nao"
      echo
    ) >> ~/.ssh/config
  fi
done

## TODO: remove lines from bashrc from old runswift stuff, path, etc.

# Set up bash
# Allow to be run as either `cd bin;./build_setup.sh` OR `./bin/build_setup.sh`
export RUNSWIFT_CHECKOUT_DIR=${PWD///bin/}
echo RUNSWIFT_CHECKOUT_DIR is $RUNSWIFT_CHECKOUT_DIR
if ! grep -q "# Robocup stuff" ~/.bashrc ; then (
echo >> ~/.bashrc
echo "# Robocup stuff" >> ~/.bashrc
echo export RUNSWIFT_CHECKOUT_DIR=\"$RUNSWIFT_CHECKOUT_DIR\" >> ~/.bashrc
echo export PATH=\"\$RUNSWIFT_CHECKOUT_DIR/bin:\$PATH\" >> ~/.bashrc
)
fi
if [[ x"$OLD_RCD" != x"$RUNSWIFT_CHECKOUT_DIR" ]]; then
  trap "myecho RUNSWIFT_CHECKOUT_DIR has changed from \'$OLD_RCD\' to \'$RUNSWIFT_CHECKOUT_DIR\'.  please be sure to reload ~/.bashrc before fixing things manually" ERR
fi

# SSH keys
ssh-keygen -l -f ~/.ssh/id_rsa.pub > /dev/null || ssh-keygen
if ! grep -qf ~/.ssh/id_rsa.pub "$RUNSWIFT_CHECKOUT_DIR"/image/home/nao/.ssh/authorized_keys; then
  echo >> "$RUNSWIFT_CHECKOUT_DIR"/image/home/nao/.ssh/authorized_keys
  echo "# $(git config user.name)'s key" >> "$RUNSWIFT_CHECKOUT_DIR"/image/home/nao/.ssh/authorized_keys
  cat ~/.ssh/id_rsa.pub >> "$RUNSWIFT_CHECKOUT_DIR"/image/home/nao/.ssh/authorized_keys
fi

########### Toolchain ##########

# CTC
mkdir -p "$RUNSWIFT_CHECKOUT_DIR"/ctc
cd "$RUNSWIFT_CHECKOUT_DIR"/ctc

export LINUX_CTC_ZIP=ctc-linux64-atom-2.1.3.3.zip
export BOOST_1550_LIBS=boost1550libs.zip
export BOOST_HEADERS=boostheaders.zip
export LIBUUID=libuuid.so.1.3.0

if [ ! -f ${LINUX_CTC_ZIP} ]; then
  echo "Please provide the toolchain zip file: $LINUX_CTC_ZIP in $RUNSWIFT_CHECKOUT_DIR/ctc"
  # Aldebaran should provide a direct download link !!!
fi

if [ ! -f ${BOOST_HEADERS} ]; then
  echo "Downloading modified boost headers"
  wget -c -N http://runswift.cse.unsw.edu.au/opennao/build-2.1.3.3/${BOOST_HEADERS}
fi

if [ ! -f ${BOOST_1550_LIBS} ];  then
  echo "Downloading pre-compiled boost 1.55.0 libs"
  wget -c -N http://runswift.cse.unsw.edu.au/opennao/build-2.1.3.3/${BOOST_1550_LIBS}
fi

if [ ! -f ${LIBUUID} ]; then
  echo "Downloading libuuid.so.1.3.0"
  wget -c -N http://runswift.cse.unsw.edu.au/opennao/build-2.1.3.3/${LIBUUID}
fi

if [ -f ${LINUX_CTC_ZIP} ]; then
  # Replace .zip with empty string
  export CTC_DIR="$RUNSWIFT_CHECKOUT_DIR"/ctc/${LINUX_CTC_ZIP/.zip/}
  [[ -d "$CTC_DIR" ]] || ( myecho Extracting cross toolchain, this may take a while... && unzip -q ${LINUX_CTC_ZIP} )
fi

if [ -f ${BOOST_HEADERS} ]; then
  export BOOST_HEADER_DIR="$RUNSWIFT_CHECKOUT_DIR"/ctc/${LINUX_CTC_ZIP/.zip/}/boost/include/boost-1_55/boost/type_traits/detail/
  unzip -j -q -o ${BOOST_HEADERS} -d ${BOOST_HEADER_DIR}
fi

if [ -f ${BOOST_1550_LIBS} ]; then
  export BOOST_1550_LIB_DIR="$RUNSWIFT_CHECKOUT_DIR"/ctc/boost_libs/
  unzip -j -q -o ${BOOST_1550_LIBS} -d ${BOOST_1550_LIB_DIR}
  # sudo unzip -j -q -o ${BOOST_1550_LIBS} -d /usr/lib/i386-linux-gnu/
fi

if ! grep -q "CTC_DIR" ~/.bashrc ; then
echo export CTC_DIR=\"$CTC_DIR\" >> ~/.bashrc
fi

echo "Changing permission on ctc dir"
chmod -R 755 ${CTC_DIR}/cross/bin
chmod -R 755 ${CTC_DIR}/cross/i686-aldebaran-linux-gnu/bin
chmod -R 755 ${CTC_DIR}/cross/libexec/

# Jayen's magic sauce
myecho rsync\'ing sysroot_legacy/usr, this may take a *long* time...
mkdir -p "$CTC_DIR"/../sysroot_legacy/usr/
rsync -a --stats --ignore-existing runswift.cse.unsw.edu.au::opennao-1.14.1/ "$CTC_DIR"/../sysroot_legacy/usr/ --exclude portage

############ Building ###########

myecho Generating Makefiles and doing the initial build
echo

# Build!
export TOOLCHAIN_FILE="$RUNSWIFT_CHECKOUT_DIR"/toolchain-2.1.cmake
for i in release relwithdebinfo; do
  cd "$RUNSWIFT_CHECKOUT_DIR"
  mkdir -p build-$i
  cd build-$i
  myecho $CTC_DIR
  cmake --debug-trycompile .. -DCMAKE_TOOLCHAIN_FILE=${TOOLCHAIN_FILE} -DCMAKE_BUILD_TYPE=$i
  make -j`nproc`
done

echo
echo All done! To build, type make -j$(nproc) in the build-release and build-relwithdebinfo directories.
echo

# Finish
echo Please close all shells.  Only new shells will have RUNSWIFT_CHECKOUT_DIR set to $RUNSWIFT_CHECKOUT_DIR
echo 'Alternatively, type . ~/.bashrc in existing shells.'
echo
