ffmpeg -i output.mp4 -vf "fps=20,split[s0][s1];[s0]palettegen[p];[s1][p]paletteuse" -loop 0 output.gif
