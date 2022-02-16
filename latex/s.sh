inotifywait -e close_write,moved_to,create -m . |
while read -r directory events filename; do
  if [ "$filename" = "0_main.pdf" ]; then
    echo 'MODIFIED'
pdftoppm 0_main.pdf __src -png -rx 300 -ry 300 && convert __src-*.png -channel RGB -negate -set filename:f '%t' ./'%[filename:f].png' && pdfjam __src-*.png --outfile output.pdf && rm __src-*.png
  fi
done



