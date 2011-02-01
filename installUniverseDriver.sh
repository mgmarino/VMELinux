#!/bin/sh
#
# Install script, this will grab most recent version
# from the repo and install it.  Repo changed to github
#
# Run (as root, from a temp directory):
#
# cd tmp
# sh /path/to/installUniverseDriver.sh install
#
#

name=universe
version=newest
FILEURL=https://github.com/mgmarino/VMELinux/tarball/master
output_file=$name-$version.tar.gz

check()
{
  lsmod | grep -q $name
  STATUS=$?
  if [ $STATUS -eq 0 ]; then
    echo "Universe module running."
  else
    echo "Universe module not running."
  fi
  return $STATUS 
}

download()
{
  echo "Downloading file at: $FILEURL"
  wget --timeout=10 --no-check-certificate $FILEURL -O $output_file
  if [ ! -f $output_file ]; then
    echo "Can't obtain $output_file, try manually copying to this directory"
    return 1
  else 
    return 0
  fi
}

inflate()
{
  echo "Inflating..."
  tar xmfz $output_file 

}

install() 
{
  my_cwd=`pwd`
  if [ ! -f $output_file ]; then
    download
    return_val=$?
    if [ "$return_val" -eq "1" ]; then
      echo "Exiting"
      exit
    fi
  fi 
  if [ ! -d $name-$version ]; then
    inflate
  fi
  cd $name-$version
  echo "Building driver..."
  cd driver && make && make install || exit $?
  cd ../
  echo "Building api..."
  cd universe_api && make && make install || exit $?
  echo "Installation done."
  cd $my_cwd
}

uninstall()
{
  my_cwd=`pwd`
  if [ ! -d $name-$version ]; then
    if [ ! -f $output_file ]; then
      echo "It doesn't look like the install folder exists, trying to obtain..."
      download
      return_val=$?
      if [ "$return_val" -eq "1" ]; then
        echo "Exiting"
        exit
      fi
    fi
    inflate
  fi
  cd $name-$version
  echo "Uninstalling driver..."
  cd driver && make uninstall || exit $?
  cd ../
  echo "Uninstalling api..."
  cd universe_api && make uninstall || exit $?
  echo "Uninstallation done."
  cd $my_cwd
}

upgrade()
{
  check
  if [ $? -ne 0 ]; then
    echo "Universe not installed, installing new version"
    cleanup
    install
  else
    echo "Universe installed, upgrading to new version"
    uninstall
    cleanup
    install
  fi
}

cleanup()
{
  rm -f $name-$version.tar.gz 
  rm -fr $name-$version
}
# See how we were called.
case "$1" in
  install)
	install
        ;;
  uninstall)
	uninstall
        ;;
  check)
	check
	;;
  cleanup)
	cleanup
	;;
  upgrade)
	upgrade
	;;
  download)
	download
	;;
  *)
        echo $"Usage: $0 {install|uninstall|upgrade|cleanup|check|download}"
        exit 1
esac

exit 0
