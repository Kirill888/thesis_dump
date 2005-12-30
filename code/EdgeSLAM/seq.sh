#!/bin/sh

echo "Looking for empty files and deleting them:"
find . -empty -exec echo removing '{}' \; -exec rm '{}' \;

echo "Building file list"
ls . > files.txt

echo "Storing Time Stamps"
echo "...left"
grep "^img-l-*" files.txt > left.files
cut -b 7-23 left.files > left.time

echo "..right"
grep "^img-r-*" files.txt > right.files
cut -b 7-23 right.files > right.time

echo "Renaming sequence:"
echo "...left"
awk '{cmd = sprintf("mv %s left_%05d.pnm",$0,NR); system(cmd); }' left.files

echo "...right"
awk '{cmd = sprintf("mv %s right_%05d.pnm",$0,NR); system(cmd); }' right.files
