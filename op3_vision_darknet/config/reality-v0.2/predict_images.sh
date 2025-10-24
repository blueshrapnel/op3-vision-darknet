#!/usr/bin/env bash
set -e

mkdir -p preds

while read -r img; do
    echo "Processing: $img"
    darknet detector test reality.data reality.cfg reality_final.weights \
        "$img" -dont_show -save_labels
    base=$(basename "$img")
    cp predictions.jpg "preds/${base%.*}-preds.jpg"
done < images.txt

echo "✅ Done. Saved annotated images in preds/"

