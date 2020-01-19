rename_pane() {
    printf '\033]2;%s\033\\' $1
}

roslaunch_with_title () {
    PKG=$1
    LAUNCH=$2
    rename_pane $PKG'/'$LAUNCH; until rostopic list ; do sleep 1; done; roslaunch --wait --disable-title $*
}

check_dir_writable() {
    DIR=$1
    if [ -w $DIR ]; 
    then echo "$DIR: WRITABLE"; 
    else echo "$DIR: NOT WRITABLE"; fi
}

