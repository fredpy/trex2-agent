
#! /bin/bash

# Number of CPU cores.
export CORES="$(grep "processor[^:]:" /proc/cpuinfo | wc -l)"
# Java folder.
export JAVA_HOME="/usr/lib/jvm/java-8-openjdk-arm64"
# Make flags.
export MAKEFLAGS="-j$CORES"

# DUNE settings.
export DUNE_GIT='https://github.com/LSTS/dune.git'
export DUNE_BRANCH='master'
export DUNE_SRC="$PWD/src/dune"
export DUNE_HOME="$PWD/dist/dune"

# EUROPA settings, using the new fixed hash_map.hh 
export EUROPA_GIT='https://github.com/zepinto/europa.git'
export EUROPA_BRANCH='entice'
export EUROPA_SRC="$PWD/src/europa"
export EUROPA_HOME="$PWD/dist/europa"
export PLASMA_HOME="$EUROPA_SRC"

# TREX settings.
export TREX_GIT='https://github.com/trygvefossum/trex2-agent'
export TREX_BRANCH='Chla'
export TREX_SRC="$PWD/src/trex"
export TREX_HOME="$PWD/dist/trex"

die()
{
    echo "ERROR: failed to $1"
    exit 1
}

################################################################################
# DUNE #
################################################################################
build_dune()
{
    if [ -d "$DUNE_SRC" ]; then
        git -C "$DUNE_HOME" clean -xdf
    else
        git clone "$DUNE_GIT" "$DUNE_SRC" || die 'download DUNE'
        git -C "$DUNE_SRC" checkout "$DUNE_BRANCH" || die 'checkout DUNE release'

    fi
    cd "$DUNE_SRC" &&
        mkdir -p build &&
        cd build &&
        cmake \
            -DSHARED=ON \
            -DCMAKE_INSTALL_PREFIX="$DUNE_HOME" \
            .. &&
        make install || die 'build DUNE'
}

################################################################################
# EUROPA 2.6 #
################################################################################
build_europa()
{
    if [ -d "$PLASMA_HOME" ]; then
        git -C "$PLASMA_HOME" clean -xdf
    else
        git clone "$EUROPA_GIT" "$EUROPA_SRC" || die 'download EUROPA'
        git -C "$EUROPA_SRC" checkout "$EUROPA_BRANCH" || die 'checkout EUROPA release'
    fi
    cd "$EUROPA_SRC" || die 'change folder to EUROPA root'

    # Fixes.
    file="$EUROPA_SRC/src/PLASMA/SWIGRules"
    grep fpermissive  "$file" > /dev/null 2>&1
    if [ $? -ne 0 ]; then
        sed 's/= -O1/= -O1 -fpermissive/g' "$file" -i
    fi

    file="$EUROPA_SRC/src/Java/PSEngine/src/psengine/PSUtil.java"
    grep 'retval.add(PSResource' "$file" > /dev/null 2>&1
    if [ $? -eq 0 ]; then
        sed -e '39,40d' "$file" -i
    fi

    # Build distribution.
    ant \
        -Djam.args="-dx" \
        -Djam.antlr.include="-I$JAVA_HOME/include" \
        -Djam.num.cores="$CORES" \
        -Djam.variant=OPTIMIZED \
        -Djam.libraries=SHARED \
        release-dist || die "build EUROPA"

    cd "$DUNE_SRC" &&
        mkdir -p build &&
        cd build &&
        cmake \
            -DSHARED=ON \
            -DCMAKE_INSTALL_PREFIX="$DUNE_HOME" \
            .. &&
        make install || die 'build DUNE'
}

################################################################################
# EUROPA 2.6 #
################################################################################
build_europa()
{
    if [ -d "$PLASMA_HOME" ]; then
        git -C "$PLASMA_HOME" clean -xdf
    else
        git clone "$EUROPA_GIT" "$EUROPA_SRC" || die 'download EUROPA'
        git -C "$EUROPA_SRC" checkout "$EUROPA_BRANCH" || die 'checkout EUROPA release'
    fi
    cd "$EUROPA_SRC" || die 'change folder to EUROPA root'

    # Fixes.
    file="$EUROPA_SRC/src/PLASMA/SWIGRules"
    grep fpermissive  "$file" > /dev/null 2>&1
    if [ $? -ne 0 ]; then
        sed 's/= -O1/= -O1 -fpermissive/g' "$file" -i
    fi

    file="$EUROPA_SRC/src/Java/PSEngine/src/psengine/PSUtil.java"
    grep 'retval.add(PSResource' "$file" > /dev/null 2>&1
    if [ $? -eq 0 ]; then
        sed -e '39,40d' "$file" -i
    fi

    # Build distribution.
    ant \
        -Djam.args="-dx" \
        -Djam.antlr.include="-I$JAVA_HOME/include" \
        -Djam.num.cores="$CORES" \
        -Djam.variant=OPTIMIZED \
        -Djam.libraries=SHARED \
        release-dist || die "build EUROPA"

    mkdir -p $EUROPA_HOME
    cd $EUROPA_HOME && unzip "$EUROPA_SRC/dist/europa-"*".zip"
}

################################################################################
# TREX #
################################################################################
build_trex()
{
    if [ -d "$TREX_HOME" ]; then
        git -C "$TREX_HOME" clean -xdf
    else
        git clone "$TREX_GIT" "$TREX_SRC" || die 'download TREX'
        git -C "$TREX_SRC" checkout "$TREX_BRANCH" || die 'checkout stable branch'
    fi

    # Fixes.
    sed 's%Dune/Dune%DUNE/DUNE%g' "$TREX_SRC/extra/third_party/lsts/CMakeLists.txt" -i

    # Build.
    cd "$TREX_SRC" || die 'change folder to TREX folder'
    mkdir -p build || die 'create build folder'
    cd build || die 'change to build folder'
    cmake \
        -DCMAKE_INSTALL_PREFIX="$TREX_HOME" \
        -DEUROPA_HINTS="$EUROPA_HOME" \
        -DWITH_EUROPA=ON \
        -DWITH_LSTS=ON \
        -DWITH_LSTS_ONBOARD=ON \
        -DWITH_LSTS_SHORESIDE=ON \
        -DWITH_PYTHON=ON \
        -DDUNE_HOME="$DUNE_HOME" .. || die "configure TREX"

    make install || die "compile TREX"
}

if [ $# -eq 0 ]; then
    build_dune
    build_europa
    build_trex
else
    for t in $*; do
        build_$t
    done
fi
