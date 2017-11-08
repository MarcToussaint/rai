###############################################################################
#
#  Copyright 2001 The University of North Carolina at Chapel Hill.
#  All Rights Reserved.
#
#  Permission to use, copy, modify OR distribute this software and its
#  documentation for educational, research and non-profit purposes, without
#  fee, and without a written agreement is hereby granted, provided that the
#  above copyright notice and the following three paragraphs appear in all
#  copies.
#
#  IN NO EVENT SHALL THE UNIVERSITY OF NORTH CAROLINA AT CHAPEL HILL BE
#  LIABLE TO ANY PARTY FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR
#  CONSEQUENTIAL DAMAGES, INCLUDING LOST PROFITS, ARISING OUT OF THE
#  USE OF THIS SOFTWARE AND ITS DOCUMENTATION, EVEN IF THE UNIVERSITY
#  OF NORTH CAROLINA HAVE BEEN ADVISED OF THE POSSIBILITY OF SUCH
#  DAMAGES.
#
#  THE UNIVERSITY OF NORTH CAROLINA SPECIFICALLY DISCLAIM ANY
#  WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
#  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.  THE SOFTWARE
#  PROVIDED HEREUNDER IS ON AN "AS IS" BASIS, AND THE UNIVERSITY OF
#  NORTH CAROLINA HAS NO OBLIGATIONS TO PROVIDE MAINTENANCE, SUPPORT,
#  UPDATES, ENHANCEMENTS, OR MODIFICATIONS.
#
#  The authors may be contacted via:
#
#  US Mail:             S. Ehmann, M. Lin
#                       Department of Computer Science
#                       Sitterson Hall, CB #3175
#                       University of N. Carolina
#                       Chapel Hill, NC 27599-3175
#
#  Phone:               (919) 962-1749
#
#  EMail:               geom@cs.unc.edu
#                       ehmann@cs.unc.edu
#                       lin@cs.unc.edu
#
###############################################################################


proc setup {} {
    wm title    . "Convex Decomposition Viewer"
    option add *Font "-adobe-helvetica-medium-r-*-*-12-*-*-*-*-*-*-*"
    set labelFont "-adobe-helvetica-bold-r-*-*-12-*-*-*-*-*-*-*"
    set labelColor lightsteelblue

    . configure -background black
    togl .view -width 800 -height 800 -rgba true \
               -ident mainRenderWin -depth true -double true -depthsize 16 \
               -alpha false -redsize 5 -greensize 5 -bluesize 5

    frame .cp1 -relief ridge -borderwidth 4 -background gray


        frame .cp1.p2 -relief groove -borderwidth 2 -background gray
            label .cp1.p2.label -relief groove -text "Display" \
                                -anchor center \
                                -background $labelColor -font $labelFont

            frame .cp1.p2.f1    -borderwidth 0

            checkbutton .cp1.p2.f1.cb0 -text "Backface Culling (b)" -anchor w \
                                       -variable backface \
                                       -selectcolor red \
                                       -command { focus .view; \
                                                  .view backfacecull }
            checkbutton .cp1.p2.f1.cb1 -text "Wireframe (w)" -anchor w \
                                       -variable wire \
                                       -selectcolor red \
                                       -command { focus .view; .view wireframe }
            checkbutton .cp1.p2.f1.cb2 -text "Color" -anchor w \
                                       -variable color \
                                       -selectcolor red \
                                       -command { focus .view; .view drawcolor }
            checkbutton .cp1.p2.f1.cb4 -text "Axes" -anchor w \
                                       -variable axes \
                                       -selectcolor red \
                                       -command { focus .view; .view drawaxes }

            button .cp1.p2.b0 -text "Center World" -anchor c \
                              -command { focus .view; .view centerw}
            button .cp1.p2.b1 -text "Set Center" -anchor c \
                              -command { focus .view; .view scenter}
            button .cp1.p2.b2 -text "Center User" -anchor c \
                              -command { focus .view; .view centeru}

        frame .cp1.p3 -relief groove -borderwidth 2 -background gray
            label .cp1.p3.label -relief groove -text "Visualize" \
                                -anchor center \
                                -background $labelColor -font $labelFont

            frame .cp1.p3.f1    -borderwidth 0
            checkbutton .cp1.p3.f1.cb0 -text "Edge Convexity" -anchor w \
                                       -variable econv \
                                       -selectcolor red \
                                       -command { focus .view; .view draweconv }
            entry .cp1.p3.f1.e1 -width 15 -relief sunken -bd 2 \
                                -textvariable which_cps -bg white -width 10


            frame .cp1.p3.f2    -borderwidth 0
            checkbutton .cp1.p3.f2.cb0 -text "Upper Leaves" -anchor w \
                                       -variable uleaves \
                                       -selectcolor red \
                                       -command { focus .view; .view drawul }

            frame .cp1.p3.f99    -borderwidth 0
            checkbutton .cp1.p3.f99.cb0 -text "Explode (x)" -anchor w \
                                       -variable explode \
                                       -selectcolor red \
                                       -command { focus .view; .view drawexpl }
            checkbutton .cp1.p3.f99.cb1 -text "Virtual Faces (v)" -anchor w \
                                       -variable vfaces \
                                       -selectcolor red \
                                       -command { focus .view; .view drawvfs }
            checkbutton .cp1.p3.f99.cb2 -text "Three Color (t)" -anchor w \
                                       -variable tcolor \
                                       -selectcolor red \
                                       -command { focus .view; .view drawtcol }
            frame .cp1.p3.f0    -borderwidth 2 -relief ridge
            set men [tk_optionMenu .cp1.p3.f0.menu editframe \
                                "Decomposition (d)" "Hierarchy (h)"]
            $men entryconfigure 0 -variable dh -value 1 -selectcolor red -command \
                     { pack forget .cp1.p3.f2; \
                       pack .cp1.p3.f1 -side top -fill x -expand true; \
                       set editframe "Decomposition"; \
                       focus .view; .view datasel; }
            $men entryconfigure 1 -variable dh -value 2 -selectcolor red -command \
                     { pack forget .cp1.p3.f1; \
                       pack .cp1.p3.f2 -side top -fill x -expand true; \
                       set editframe "Hierarchy"; \
                       focus .view; .view datasel }

        button .cp1.quit -text "Quit" -anchor c -command exit

    pack .view -side left
    pack .cp1 -side top -fill x -fill y -expand true


    pack .cp1.p2 -side top -fill x
    pack .cp1.p2.label -side top -fill x
    pack .cp1.p2.f1 -side left -fill x -expand true
    pack .cp1.p2.f1.cb0 -side top -fill x -expand true
    pack .cp1.p2.f1.cb1 -side top -fill x -expand true
    pack .cp1.p2.f1.cb2 -padx 15 -side top -fill x -expand true
    pack .cp1.p2.f1.cb4 -side top -fill x -expand true
    pack .cp1.p2.b0 -side top -fill x -expand true
    pack .cp1.p2.b1 -side top -fill x -expand true
    pack .cp1.p2.b2 -side top -fill x -expand true

    pack .cp1.p3 -side top -fill x
    pack .cp1.p3.label -side top -fill x -expand true
    pack .cp1.p3.f99 -side top -fill x -expand true
    pack .cp1.p3.f99.cb0 -side top -fill x -expand true
    pack .cp1.p3.f99.cb1 -side top -fill x -expand true
    pack .cp1.p3.f99.cb2 -side top -fill x -expand true
    pack .cp1.p3.f0 -side top -fill x -expand true
    pack .cp1.p3.f0.menu -side top -expand true

    pack .cp1.p3.f2 -side top -fill x -expand true
    pack .cp1.p3.f2.cb0 -side top -fill x -expand true
    pack forget .cp1.p3.f2

    pack .cp1.p3.f1 -side top -fill x -expand true
    pack .cp1.p3.f1.cb0 -side top -fill x -expand true
    pack .cp1.p3.f1.e1 -side top -fill x -expand true

    pack .cp1.quit -side bottom -fill x

    # Text field keys
    bind .cp1.p3.f1.e1 <Key-Return> { global which_cps; focus .view; \
                                      .view drawcps $which_cps }
    # Draw area keys
    bind .view <Key-a> { .view a_key }
    bind .view <Key-b> { .view b_key }
    bind .view <Key-c> { .view c_key }
    bind .view <Key-C> { .view C_key }
    bind .view <Key-d> { .view d_key }
    bind .view <Key-h> { .view h_key }
    bind .view <Key-j> { .view j_key }
    bind .view <Key-k> { .view k_key }
    bind .view <Key-q> { .view q_key }
    bind .view <Key-s> { .view s_key }
    bind .view <Key-t> { .view t_key }
    bind .view <Key-v> { .view v_key }
    bind .view <Key-w> { .view w_key }
    bind .view <Key-x> { .view x_key }

    bind .view <Key-minus> { .view minus_key }
    bind .view <Key-underscore> { .view minus_key }
    bind .view <Key-equal> { .view plus_key }
    bind .view <Key-plus> { .view plus_key }

    # Draw area mouse ops


    # button 1
    bind .view <ButtonPress-1> { focus .view; .view mouse1Down %x %y }
    bind .view <Shift-ButtonPress-1> { focus .view; \
                                          .view mouse1ShiftDown %x %y }
    bind .view <ButtonRelease-1> { .view mouse1Up }
    bind .view <B1-Motion> { .view mouseDrag %x %y }

    # button 2
    bind .view <ButtonPress-2> { focus .view; .view mouse2Down %x %y }
    bind .view <Shift-ButtonPress-2> { focus .view; \
                                          .view mouse2ShiftDown %x %y }
    bind .view <ButtonRelease-2> { .view mouse2Up }
    bind .view <B2-Motion> { .view mouseDrag %x %y }

    # button 3
    bind .view <ButtonPress-3> { focus .view; .view mouse3Down %x %y }
    bind .view <Shift-ButtonPress-3> { focus .view; \
                                            .view mouse3ShiftDown %x %y }
    bind .view <ButtonRelease-3> { .view mouse3Up }
    bind .view <B3-Motion> { .view mouseDrag %x %y }
}

# Start execution here
setup

# default values
set which_cps "A"

# set focus to the draw area (togl widget)
# Actually, this does not do anything because the window is not up
focus .view


