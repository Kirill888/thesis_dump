MAIN = thesis
TEXES = $(MAIN).tex notation_definitions.tex chpt_LiteratureReview.tex \
        chpt_FastSLAM.tex chpt_MapStructure.tex chpt_Mapping.tex\
        chpt_LoopClosing.tex chpt_Summary.tex  \
        chpt_ExperimentLaser.tex chpt_ExperimentVision.tex \
	chpt_Conclusion.tex

BIBS = my_ref.bib bio_ref.bib ref_slam.bib

PICS_DIR = Pics

#Figures in .fig format
FIGSf = fig_transition1.fig fig_transition2.fig fig_transition3.fig\
        fig_example1.fig fig_example2.fig fig_example3.fig fig_example4.fig\
        fig_map_structure.fig fig_transition_model.fig fig_corner_example.fig\
        fig_revisit1.fig fig_revisit2.fig fig_revisit3.fig fig_revisit4.fig\
        fig_convex_reg.fig fig_corner_types.fig fig_inconsistent_map.fig\
	fig_bft.fig fig_mirror_sets.fig

FIGS = $(FIGSf:%=$(PICS_DIR)/%)

#EPS files not derived from .fig
EPSf = map_example_indoor.eps corner_scan_example_a.eps \
       corner_scan_example_b.eps corner_scan_example_c.eps \
       corner_scan_example_d.eps map_example_outdoor.eps\
       example_edge_detector.eps \
       corner_map_100p.eps corner_odo_100p.eps corner_odo_all_100p.eps\
       corner_map_300p.eps corner_odo_300p.eps corner_odo_all_300p.eps\
       corners_execution_time.eps\
       edge_map_100p.eps edge_odo_100p.eps edge_odo_all_100p.eps\
       edge_map_300p.eps edge_odo_300p.eps edge_odo_all_300p.eps\
       edge_map_300p_broken.eps edge_odo_300p_broken.eps \
       edge_map_300p_broken2.eps edge_odo_300p_broken2.eps \
       trees_map_100p.eps trees_odo_100p.eps trees_odo_all_100p.eps\
       trees_map_300p.eps trees_odo_300p.eps trees_odo_all_300p.eps\
       xr4000_raw_odo.eps hump.eps hump_xy.eps post_example.eps post_obs.eps\
       post_obsodo.eps post_odo.eps post_odo_ekf.eps post_odo_pf.eps\
       post_odo_pf_.eps subway_map.eps rsise_level3_map.eps\
       barrel_distortion.eps undist_barrel_distortion.eps


EPS = $(EPSf:%=$(PICS_DIR)/%)
EPS += $(FIGS:%.fig=%.eps)

PICS =

SRC_SPECIAL = -src

RSYNC=rsync
RSYNC_ARGS=-rtv --modify-window=2 -u
RSYNC_ARGS+=--exclude-from=exclude.txt
RSYNC_ARGS+=--exclude=$(MAIN).tgz
#RSYNC_ARGS+=-n
USB=/mnt/sda1/Thesis

default:  $(MAIN).dvi

tar: $(MAIN).tgz

pdf: $(MAIN).pdf

$(MAIN).dvi: $(TEXES) $(MAIN).bbl $(EPS) anuthesis.sty
	latex $(SRC_SPECIAL) $(MAIN).tex | tee $(MAIN).out
	-fgrep -si "Reference .* undefined" $(MAIN).out ||\
                 fgrep -si "Rerun to get cross-references" $(MAIN).out &&\
                 latex $(SRC_SPECIAL) $(MAIN).tex
	@rm -f $(MAIN).out


$(MAIN).bbl: $(BIBS) $(EPS) $(TEXES)
	latex $(SRC_SPECIAL) $(MAIN).tex
	bibtex $(MAIN)

%.eps: %.fig
	fig2dev -L eps $< $@

%.pdf: %.eps
	epstopdf --outfile=$@ $<

%.ps: %.dvi
	dvips -o $@ $<

$(MAIN).pdf: $(TEXES) $(MAIN).bbl $(EPS) $(EPS:.eps=.pdf)
	pdflatex $(MAIN).tex



$(MAIN).tgz: $(TEXES) $(BIBS) $(EPS) $(FIGS) Makefile
	tar -cvzf $(MAIN).tgz --exclude-from exclude.txt --exclude $@ ./


put:
	@echo Syncing local "=>" $(USB)
	$(RSYNC) $(RSYNC_ARGS) ./ $(USB)/
get:
	@echo Syncing $(USB) "=>" local
	$(RSYNC) $(RSYNC_ARGS) $(USB)/ ./

diff:
	diff ./ $(USB)/ | less
clean:
	rm -f $(MAIN).ps $(MAIN).pdf $(MAIN).dvi\
	$(MAIN).aux $(MAIN).log $(MAIN).bbl $(MAIN).blg
