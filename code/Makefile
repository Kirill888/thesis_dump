RSYNC=rsync
RSYNC_ARGS=-rtv --modify-window=2
#RSYNC_ARGS+=-n
RSYNC_ARGS+=--exclude-from=exclude.txt

USB=/mnt/sda1/ShinySLAM

put:
	$(RSYNC) $(RSYNC_ARGS) ./ $(USB)/

get:
	$(RSYNC) $(RSYNC_ARGS) $(USB)/ ./

