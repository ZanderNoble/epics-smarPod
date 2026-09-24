#!/bin/sh
# install script for SmarAct Software for Linux
# (c) 2011-2021 SmarAct GmbH

fail() {
    ARG="$1"
    shift
    log 'FAIL' "$ARG" "$@"
    printf "error: $ARG\n" "$@"
    return 1
} >&2

install_file() {
    if ! umask 0022
    then
        fail 'Could not adjust umask'
        return
    fi
    if ! mkdir -p "$(dirname "$2")"
    then
        fail 'Cannot create directory for %s' "$2"
        return
    fi
    if ! umask 0222
    then
        fail 'Could not adjust umask'
        return
    fi
    case "$2" in
        /*)
            if [ -n "${FORCE_ABSOLUTE+x}" ]
            then
                if ! cp -Pf "$1" "$2"
                then
                    fail 'Could not force install %s' "$2"
                    return
                fi
            else
                if [ -f "$2" ]
                then
                    printf 'info: %s is already installed, skipping\n' "$2"
                elif ! cp -P "$1" "$2"
                then
                    fail 'Could not install %s' "$2"
                    return
                fi
            fi
            ;;
        *)
            if [ -f "$2" ]
            then
                fail "%s is already installed" "$2"
                return
            fi
            if ! cp -P "$1" "$2"
            then
                fail 'Could not install %s' "$2"
                return
            fi
    esac
}

install_executable() {
    TARGET_FILE="$DIR"/"$(dirname "$1")"/."$(basename "$1")".real
    if ! install_file "$EXTRACT_DIR"/files/"$1" "$TARGET_FILE"
    then
        return 1
    fi
    if ! cat >"$DIR"/"$1" <<EOF
#!/bin/sh
export LD_LIBRARY_PATH="$(realpath "$DIR")/lib" FONTCONFIG_FILE="$(realpath "$DIR")/etc/smaract-fonts.conf"
exec "$(realpath "$TARGET_FILE")"
EOF
    then
        log 'ERROR' 'Could not create executable wrapper for %s' "$TARGET_FILE"
        return 1
    fi
    if ! chmod +x "$DIR"/"$1"
    then
        log 'ERROR' 'Could not set executable bit on wrapper' "$DIR"/"$1"
        return 1
    fi
    return 0
}

increment_refcount() {
    read -r REFC <"$DEPDIR"/refcount \
        && printf '%d\n' "$((REFC + 1))" >"$DEPDIR"/refcount
}

install_pack() {

unset -v FORCE_ABSOLUTE SIDE_INSTALL
DIR="$1"
PACK="$2"
SIDE_INSTALL=$3
FORCE_ABSOLUTE=$4

if [ -z "$PACK" ]
then
    fail "No package specified"
    return
fi

if [ ! -f "$PACK" ]
then
    fail 'Pack "%s" cannot be found' "$PACK"
    return
fi

EXTRACT_DIR="$(mktemp -d --tmpdir "pack-XXXXXXXX")"
tmpfiles="$tmpfiles $EXTRACT_DIR"

if ! tar -xf "$PACK" -C "$EXTRACT_DIR"
then
    fail 'Cannot unpack pack file'
    return
fi

if [ ! -f "$EXTRACT_DIR"/info ]
then
    fail 'Pack does not contain an info file'
    return
fi

unset -v name deps files patch_rpath
if ! . "$EXTRACT_DIR"/info
then
    fail 'Could not read configuration file %s' "$EXTRACT_DIR/info"
    return
fi

if [ -n "${patch_rpath+x}" ]
then
    log 'WARN' 'Package %s has patch_rpath set which is deprecated' "$name"
fi

if [ -z "$name" ]
then
    fail 'Pack does not contain a name'
    return
fi

PKGDIR="$DIR/var/smaract/$name"

if [ -f "$PKGDIR/info" ]
then
    if ! ( unset -v name deps weak_deps files
           if ! . "$PKGDIR"/info
           then
               fail 'Could not read configuration file'
               exit 1
           fi
           if ! { cd "$DIR" && rm -f $files; }
           then
               fail 'Could not delete old files'
               exit 1
           fi )
    then
        fail 'Failed to scrape old %s' "$name"
        return
    fi
    log 'INFO' 'Scraped old %s installation before reinstalling' "$name"
    if [ -f "$PKGDIR/refcount" ]
    then
        if [ -n "$SIDE_INSTALL" ]
        then
            log 'WARN' 'Package %s is already installed, reinstalling' "$name"
            printf 'warning: Package %s is already installed, reinstalling\n' "$name"
        else
            log 'INFO' 'Marking %s as manually installed\n' "$name"
            if ! [ -f "$PKGDIR"/manual_install ]
            then
                printf 'Marking %s as manually installed\n' "$name" >&2
                if ! touch "$PKGDIR/manual_install"
                then
                    fail 'Could not mark %s as manually installed' "$name"
                    return
                fi
            fi
        fi
    else
        fail 'Metadata for %s seems malformed' "$name"
        return
    fi
    REINSTALL=1
else
    REINSTALL=0
fi

if ! OLD_MASK="$(umask)"
then
    fail 'Could not record current umask'
    return
fi

for i in $files
do
    case "$i" in
        /*)
            log 'INFO' 'Installing %s to %s' \
                "$(basename "$i")" "$(dirname "$i")"
            install_file "$EXTRACT_DIR/files/${i#/}" "$i"
            ;;
        bin/*)
            if [ -x "$EXTRACT_DIR/files/$i" ]
            then
                install_executable "$i"
            else
                install_file "$EXTRACT_DIR/files/$i" "$DIR/$i"
            fi
            ;;
        *)
            install_file "$EXTRACT_DIR/files/$i" "$DIR/$i"
            ;;
    esac
done

if ! umask "$OLD_MASK"
then
    fail 'Could not restore old umask'
    return
fi

if ! mkdir -p "$PKGDIR"
then
    fail 'Could not create folder for packet information'
    return
fi
if ! cp -a "$EXTRACT_DIR"/info "$PKGDIR/info"
then
    fail 'Could not install info file'
    return
fi

if [ "$REINSTALL" -eq 1 ]
then
    return 0
fi

if [ ! -f "$PKGDIR/refcount" ]
then
    if ! echo 0 >"$PKGDIR/refcount"
    then
        fail 'Could not write refcount'
        # TODO: Roll back
        return
    fi
fi
if [ -z "$SIDE_INSTALL" ]
then
    if ! touch "$PKGDIR/manual_install"
    then
        fail 'Could not mark the package manually installed'
        # TODO: Roll back
        return
    fi
fi

for i in $deps
do
    if [ "$i" = "$name" ]
    then
        fail 'Pack specifies itself as a dependency'
        return
    fi
    DEPDIR="$DIR/var/smaract/$i"
    if [ ! -d "$DEPDIR" ] || [ ! -f "$DEPDIR"/info ] || [ ! -f "$DEPDIR"/refcount ]
    then
        fail 'Cannot install %s: dependency %s is not installed or corrupted' \
             "$name" "$i"
        return
    fi
    if ! increment_refcount
    then
        fail 'Cannot increment refcount for %s' "$i"
        # TODO: Roll back
        return
    fi
done

for i in $weak_deps
do
    if [ "$i" = "$name" ]
    then
        fail 'Pack specifies itself as a weak dependency'
        return
    fi
    DEPDIR="$DIR"/var/smaract/"$i"
    if [ ! -d "$DEPDIR" ]
    then
        continue
    fi
    if [ ! -f "$DEPDIR"/info ] || [ ! -f "$DEPDIR"/refcount ]
    then
        fail 'Packet %s is corrupted' "$i"
        return
    fi
    if ! increment_refcount
    then
        fail 'Cannot increment refcount for %s' "$i"
        return
    fi
    if ! echo "$i"
    then
        fail 'Cannot write weak_dep'
        return
    fi
done >"$PKGDIR"/weak_deps

return 0

}

uninstall_pack() {

DIR="$1"
PACK="$2"
SIDE_INSTALL=$3

if [ -z "$PACK" ]
then
    fail 'No package specified'
    return
fi

PKGDIR="$DIR"/var/smaract/"$PACK"

if [ ! -d "$PKGDIR" ]
then
    fail 'Could not find metadata for %s' "$PACK"
    return
fi

if [ ! -f "$PKGDIR"/info ]
then
    fail 'Pack does not contain an info file'
    return
fi

# refcount is needed in both following if branches
if ! read -r REFC <"$PKGDIR"/refcount
then
    fail 'Could not read refcount for %s' "$PACK"
    return
fi

if [ -z $SIDE_INSTALL ]
then
    if [ ! -f "$PKGDIR"/manual_install ]
    then
        printf 'warning: Packet %s was not installed manually before\n' \
               "$PACK" >&2
        log 'INFO' 'Packet %s should be manually removed but is not manually installed' \
            "$PACK"
        return 0
    fi
    if ! rm "$PKGDIR"/manual_install
    then
        fail 'Could not remove manual installation mark'
        return
    fi
    if [ "$REFC" -ge 1 ]
    then
        printf 'Removed manual installation mark from %s\n' "$PACK"
        return 0
    fi
else
    if ! printf '%d\n' "$((REFC - 1))" >"$PKGDIR/refcount"
    then
        fail 'Could not write refcount for %s' "$PACK"
        return
    fi
    if [ $((REFC - 1)) -ge 1 ] || [ -f "$PKGDIR"/manual_install ]
    then
        return 0
    fi
fi

unset -v name deps weak_deps files
if ! . "$PKGDIR"/info
then
    fail 'Cannot read configuration file %s' "$PKGDIR/info"
    return
fi

CODE=0
for i in $deps
do
    if [ "$i" = "$name" ]
    then
        fail 'Pack specifies itself as a dependency'
        CODE=1
        continue
    fi
    # Run in subshell to keep our variables
    if ! (uninstall_pack "$DIR" "$i" t)
    then
        fail 'Could not remove %s' "$i"
        CODE=1
        continue
    fi
done
while read -r weak_dep
do
    if [ "$weak_dep" = "$name" ]
    then
        fail 'Pack specifies itself as a dependency'
        CODE=1
        continue
    fi
    if ! (uninstall_pack "$DIR" "$weak_dep" t)
    then
        fail 'Could not remove %s' "$i"
        CODE=1
        continue
    fi
done <"$PKGDIR"/weak_deps

unset -v NOEXIT

if ! (cd "$DIR" && rm -f $files)
then
    fail 'Could not remove files'
    return
fi
if ! rm -rf "$PKGDIR"
then
    fail 'Could not delete package directory'
    return
fi
return $CODE

}

# log "INFO" "Use me like %s" "this"
# The specification may not be longer than 8 chars
log_impl() {
    SPEC="$1"
    FMT="$2"
    shift 2
    if ! DATE="$(date "+%Y-%m-%d %T")"
    then
        DATE="<nodate>"
    fi
    printf "%-19s %8s: $FMT\n" "$DATE" "$SPEC" "$@"
}

showhelp() {
    if ! BASENAME="$(basename "$0")"
    then
        echo "error: Could not determine basename of the application" >&2
        BASENAME="install.sh"
    fi
    printf 'Usage: %s [OPTIONS]... [COMPONENTS]\n
Options:
  -h | --help            Display a short help text and quit.
  --uninstall            Uninstall software.
  --version              Display version and quit.
  --license v            With `--license accept` you confirm that you have read
                         the SmarAct Software License Agreement (EULA) and
                         accept it.
                         It must be accepted in order to install and use the
                         software.
                         The EULA text is included in the software package.
                         If it is not included, please contact SmarAct.
  --installdir p         Use path p as the base of the installation.
  --list-components      List all components available and exit.
  --arch a               The target architecture a: x86 or x86_64.
                         The system architecture is used if omitted.
  --log-file f           File to write the installation log to.
  --show-installdir-log  Prints out the collected logs for the installdir and
                         exits.
COMPONENTS:              List of comma separated components to install.
                         The installation of previously selected components can
                         be revoked by prefixing it with "no-", e.g. "no-comp"
                         to prevent installation of "comp".\n' \
           "$BASENAME" | cat "$SRCPATH"/version-* -
}

unset -v side_install

load_config() {
    unset -v dep_comps deps packs implies description
    if ! . "$SRCPATH"/.resources/conf/"$1".conf
    then
        echo 'Error reading configuration file' >&2
        "$LOG" 'FAIL' 'Cannot read configuration for %s' "$1"
        return 1
    fi
    "$LOG" 'LOAD' 'Configuration file %s' "$SRCPATH"/.resources/conf/"$1".conf
    return 0
}

with_side_install() {
    if [ -z ${side_install+x} ]
    then
        side_install=1
        "$@"
        CODE=$?
        unset -v side_install
        [ $CODE -eq 0 ] || return $CODE
    else
        "$@" || return $CODE
    fi
    return 0
}

without_side_install() {
    if [ -n "${side_install+x}" ]
    then
        unset -v side_install
        "$@"
        CODE=$?
        side=install=1
        [ $CODE -eq 0 ] || return $CODE
    else
        "$@" || return $CODE
    fi
    return 0
}

install_component_impl() {
    for i in $2
    do
        log 'INFO' 'Installing %s as a dependency of %s' "$i" "$1"
        with_side_install install_component "$i" || return $?
    done
    for i in $5
    do
        for j in $NO_COMPONENTS
        do
            if [ "$j" = "$i" ]
            then
                log 'INFO' 'Implied component %s was explicitly disabled' "$i"
                continue 2
            fi
        done
        log 'INFO' 'Installing %s as implied by %s' "$i" "$1"
        with_side_install install_component "$i" || return $?
    done
    for i in $3
    do
        log 'INSTALL' '%s as a dependency of %s' "$i" "$1"
        if ! install_pack "$IPATH" "$SRCPATH"/.resources/packs/"$i"_"$ARCH".pack t
        then
            echo "Error installing dependency $i" >&2
            log 'FAIL' 'Cannot install dep %s for component %s' "$i" "$1"
            return 1
        fi
    done
    for i in $4
    do
        log 'INSTALL' '%s as a pack of %s' "$i" "$1"
        if ! install_pack "$IPATH" "$SRCPATH"/.resources/packs/"$i"_"$ARCH".pack \
             $side_install
        then
            echo "Error installing pack $i" >&2
            log 'FAIL' 'Cannot install pack %s for component %s' "$i" "$1"
            return 1
        fi
    done
    return 0
}


install_component() {
    # exit if already installed
    for i in $INSTALLED_COMPONENTS
    do
        if [ "$i" = "$1" ]
        then
            log 'SKIP' '%s was already installed' "$i"
            return 0
        fi
    done

    log 'INSTALL' 'Component %s' "$1"
    load_config "$1" || return 1
    set -- "$1" "$dep_comps" "$deps" "$packs" "$implies"
    if [ -z ${side_install+x} ]
    then
        log 'INFO' '%s is a main installation target' "$1"
        install_component_impl "$@" || return $?
    else
        FOUND=0
        for i in $COMPONENTS
        do
            if [ "$i" = "$1" ]
            then
                FOUND=1
                break
            fi
        done
        if [ "$FOUND" -eq 1 ]
        then
            log 'INFO' '%s is a main install' "$1"
            without_side_install install_component_impl "$@" \
                                 || return $?
        else
            log 'INFO' '%s is a side install' "$1"
            install_component_impl "$@" || return $?
        fi
    fi


    INSTALLED_COMPONENTS="$1 $INSTALLED_COMPONENTS"
    log 'DONE' 'Component %s handled' "$1"
    return 0
}

# Helper for uninstall_component
uninstall_list() {
    first_round=yes
    for arg
    do
        if [ -n "$first_round" ]
        then
            set --
            first_round=
        fi
        set -- "$arg" "$@"
    done
    while [ $# -gt 0 ]
    do
        if ! uninstall_pack "$IPATH" "$1"
        then
            echo "Error uninstalling $i" >&2
            log 'FAIL' 'Cannot uninstall pack %s' "$1"
            return 1
        fi
        log 'REMOVE' 'Pack %s' "$1"
        shift
    done
}

uninstall_component() {
    for i in $INSTALLED_COMPONENTS
    do
        if [ "$i" = "$1" ]
        then
            return 0
        fi
    done

    if ! load_config "$1"
    then
        return 1
    fi

    if ! uninstall_list $packs
    then
        return 1
    fi
    INSTALLED_COMPONENTS="$INSTALLED_COMPONENTS $1"
    return 0
}

self="${0#./}"
base="${self%/*}"

SRCPATH=""
if [ "$base" = "$self" ]; then
    SRCPATH="$(pwd)"
else
    SRCPATH="$(pwd)/$base"
fi

DO_UNINSTALL=0
IPATH="/usr/local"
LICENSE_VALUE="noaccept"
ARCH="$(uname -m)"
LIST_COMPONENTS=0
SHOW_HELP=0
unset -v LOGFILE SHOW_INSTALLDIR_LOG COMPONENTS

ARGTEXT="$*"

while [ $# -gt 0 ]
do
    case "$1" in
        --uninstall)
            DO_UNINSTALL=1
            ;;
        --version)
            cat "$SRCPATH"/version-*
            exit 0
            ;;
        -h | --help)
            SHOW_HELP=1
            ;;
        --license)
            if [ $# -eq 1 ]; then
                { printf "Expected value after --license\n\n"
                  showhelp; } >&2
                exit 1
            fi
            LICENSE_VALUE="$2"
            shift
            ;;
        --arch)
            if [ $# -eq 1 ]; then
                { printf "Expected architecture after --arch\n\n"
                  showhelp; } >&2
                exit 1
            fi
            if [ "$2" != "x86" ] && [ "$2" != "x86_64" ]
            then
                { printf 'error: Unknown architecture: %s\n' "$2"
                  showhelp; } >&2
                exit 1
            fi
            ARCH="$2"
            shift
            ;;
        --list-components)
            LIST_COMPONENTS=1
            ;;
        --log-file)
            if [ $# -eq 1 ]; then
                { printf "Expected file name after --log-file\n\n"
                  showhelp; } >&2
                exit 1
            fi
            LOGFILE="$2"
            shift
            ;;
        --show-installdir-log)
            SHOW_INSTALLDIR_LOG=1
            ;;
        --installdir)
            if [ $# -eq 1 ]; then
                { printf "Expected directory list after --installdir\n\n"
                  showhelp; } >&2
                exit 1
            fi
            IPATH="$2"
            shift
            ;;
        -*)
            { printf 'error: %s is not a valid option\n\n' "$1"
              showhelp; } >&2
            exit 1
            ;;
        *)
            if [ -n "${COMPONENTS+x}" ]
            then
                { printf '%s: "%s" vs. "%s"\n%s\n\n' \
                         'Components specified multiple times' \
                         "$COMPONENTS" "$1" \
                         'Use commas to separate components'
                  showhelp;} >&2
                exit 1
            fi
            COMPONENTS="$1"
            ;;
    esac
    shift
done

if [ -n "${SHOW_INSTALLDIR_LOG+x}" ]
then
    if [ ! -f "$IPATH"/var/smaract/log.txt ]
    then
        printf 'error: %s does not seem to contain a global log\n' "$IPATH" >&2
        exit 1
    fi
    cat "$IPATH"/var/smaract/log.txt
    exit
fi

log_redirect() { cat >&3; }
log() { log_impl "$@" >&3; } # optimization: Don't use cat

if [ -n "${LOGFILE+x}" ]
then
    if { DIRNAME="$(dirname "$LOGFILE")" && mkdir -p "$DIRNAME" \
             && exec 4>>"$LOGFILE"; }
    then
        log_redirect() {
            while IFS="" read -r LINE
            do
                echo "$LINE" >&3 && echo "$LINE" >&4 || return $?
            done
        }
        log() { log_impl "$@" | log_redirect; }
    else
        unset -v LOGFILE
        echo "error: Could not open logfile $LOGFILE" >&2
    fi
fi

if ! COMPONENT_LIST="$(echo "$SRCPATH"/.resources/conf/*.conf \
                       | sed "s|$SRCPATH/.resources/conf/\([^.]*\).conf|\1\n|g")"
then
    echo "Component listing failed" >&2
    exit 1
elif [ "$COMPONENT_LIST" = "*" ]
then
     COMPONENT_LIST=""
fi

list_components() {
    echo 'Available components: '
    len=0
    for i in $COMPONENT_LIST
    do
        olen=${#i}
        if [ $olen -gt $len ]
        then
            len=$olen
        fi
    done
    for i in $COMPONENT_LIST
    do
        LOG=":"
        load_config "$i"
        if [ -z ${description+x} ]
        then
            printf '%s %s\n' '-' "$i"
        else
            printf "%s %-${len}s: %s\n" '-' "$i" "$description"
        fi
    done
}

if [ $SHOW_HELP -eq 1 ]
then
    showhelp
    list_components
    exit 0
fi

if [ $LIST_COMPONENTS -eq 1 ]
then
    list_components
    exit 0
fi

case "$ARCH" in
    x86_64) ARCH='x64';;
    i[3-6]86) ARCH='x86';;
    *)  echo "Unknown or unsupported architecture." >&2
        return 1;;
esac

if [ "${IPATH#/}" = "$IPATH" ]
then
    ABS_IPATH="$PWD/$IPATH"
else
    ABS_IPATH="$IPATH"
fi

# construct components
if ! COMPONENTS="$(printf '%s' "$COMPONENTS" | tr ',' ' ')"
then
    echo "Could not transform component list" >&2
    exit 1
fi

unset -v DEFAULT_COMPS
if [ -f "$SRCPATH"/.resources/defaults ]
then
    if ! DEFAULT_COMPS="$(cat "$SRCPATH"/.resources/defaults)"
    then
        echo "Could not read default components" >&2
    fi
fi

BACKUP="$COMPONENTS"
COMPONENTS=
NO_COMPONENTS=
for i in $BACKUP $DEFAULT_COMPS
do
    case "$i" in
        no-*)
            NO_COMPONENTS="$NO_COMPONENTS ${i#no-}"
            ;;
        *)
            COMPONENTS="$COMPONENTS $i"
    esac
done
BACKUP="$COMPONENTS"
COMPONENTS=
for i in $BACKUP
do
    for j in $NO_COMPONENTS
    do
        if [ "$i" = "$j" ]
        then
            continue 2
        fi
    done
    COMPONENTS="$COMPONENTS $i"
done
unset -v BACKUP

# Check for unknown components
for i in $COMPONENTS $NO_COMPONENTS
do
    for j in $COMPONENT_LIST
    do
        if [ "$j" = "$i" ]
        then
            continue 2
        fi
    done
    { set -- "$i" # prevent clobbering by list_components
      list_components
      echo "error: There is no component named $1"
    } >&2
    exit 1
done

INSTALLED_COMPONENTS=""
if [ "$DO_UNINSTALL" -eq 0 ]
then
    if [ "$LICENSE_VALUE" != "accept" ]; then
        printf '*******************************************************
Could not install:
In order to install this software, the license agreement
must be accepted. Please read the installer help.
*******************************************************\n' >&2
        exit 1
    fi
    if [ -z "$COMPONENTS" ]
    then
        { list_components
          printf '\n*******************************************************
Could not install:
No component is selected.
There is nothing to install.
*******************************************************\n'
        } >&2
        exit 1
    fi

    if ! mkdir -p "$IPATH"
    then
        echo "Cannot create installation directory" >&2
        exit 1
    fi
    OP_NAME=Installing
    OP=install_component
else
    if [ -z "$COMPONENTS" ]
    then
        printf '*******************************************************
Could not install:
No component is selected.
There is nothing to remove.
*******************************************************\n' >&2
        exit 1
    fi

    if [ ! -d "$IPATH" ]
    then
        echo "Installation directory does not exist" >&2
        exit 1
    fi
    OP_NAME=Uninstalling
    OP=uninstall_component
fi

# Setup fd 3 for logging
if ! mkdir -p "$IPATH/var/smaract"
then
    echo "error: Could not create $IPATH/var/smaract" >&2
    exit 1
fi
if ! exec 3>>"$IPATH/var/smaract/log.txt"
then
    echo "error: Could not create log.txt" >&2
    exit 1
fi

{ echo "########################################"
  log_impl 'ARGS' '%s' "$ARGTEXT"
  log_impl 'PATH' '%s' "$ABS_IPATH"
  log_impl 'PRODUCT' '%s' 'The following is the product information'
  for i in "$SRCPATH"/version-*
  do
      if [ "$i" = "$SRCPATH/version-*" ]
      then
          echo "error: No version file found in the installer" >&2
          log_impl 'ERROR' '%s' "Installer does not contain a version file"
          continue
      fi
      while IFS="" read -r LINE
      do
          printf "%20s%s\n" "" "$LINE"
      done <"$i"
  done
  if ! COMPONENTS="$(printf '%s' "$COMPONENTS" | tr ',' ' ')"
  then
      echo "Could not transform component list" >&2
      log_impl 'FAIL' 'Could not transform component list'
  fi
  log_impl 'INFO' 'Components after filtering: %s' "$COMPONENTS"
  log_impl 'INFO' 'Components explicitly disabled: %s' "$NO_COMPONENTS"
} | log_redirect
LOG=log

if ! trap 'rm -rf $tmpfiles' INT TERM EXIT
then
    log 'WARN' 'Could not setup temporary file cleaning'
fi

for comp in $COMPONENTS
do
    if ! "$OP" "$comp"
    then
        printf '%s %s for %s in %s: failed\n' "$OP_NAME" "$COMPONENTS" "$ARCH" "$IPATH" >&2
        exit 1
    fi
done

if command -v ldconfig >/dev/null
then
    if ! command ldconfig
    then
        echo "warning: Could not run ldconfig" >&2
        log 'WARN' 'Could not run ldconfig'
    fi
else
    log 'INFO' 'ldconfig was not found'
fi

printf '%s %s for %s in %s: done\n' "$OP_NAME" "$COMPONENTS" "$ARCH" "$IPATH" >&2
log 'SUCCESS' 'Installer finished'
exit 0
