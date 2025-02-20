BEGIN {
   posX = 0
   posY = 1600
   FS = "[ =]"
   ret = 0
}

/A: position=/{
   posX = $6
}
/B: position=/{
   posY = $6+1600
}
/: position/ {
   r = sqrt(posX*posX+posY*posY)
   if ((r > 1602) || (r < 1597)) {
      print(posX,posY,r)
      ret = -1
   }
}

END {
   exit(ret)
}
