#!/bin/bash

declare -a WorkFiles=(
    "Apps/DefSLAM"
    "Apps/DefSLAMGT"
    "Apps/run.sh"
    "Apps/recompile.sh"
    "Thirdparty/g2o/lib/libg2o.so"
    "skip.sh"
)

skip_files()
{
    echo "git update-index --skip-worktree ..."
    for f in ${WorkFiles[@]};
    do
        git update-index --skip-worktree $f
    done
    git ls-files -v . | grep ^S
}

unskip_files()
{
    echo "git update-index --no-skip-worktree ..."
    for f in ${WorkFiles[@]};
    do
        git update-index --no-skip-worktree $f
    done
    git status
}


echo "Skip worktree for the following files? y/n"
for val in ${WorkFiles[@]};
do
    echo -e "\t $val"
done

echo -n ">> "
read ANS
case $ANS in
    y) skip_files;;
    n) unskip_files;;
    *) echo "y/n?"''
esac
