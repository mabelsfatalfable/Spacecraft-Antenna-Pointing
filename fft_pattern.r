library(jpeg)
library(graphics)
library(rgl)

img <- readJPEG(choose.files())

img_ff <- fft(img)
phase <- atan(Im(img_ff)/Re(img_ff))
magnitude <- sqrt(Re(img_ff)^2 + Im(img_ff)^2)



# FFT SHIFT
fftshift <- function(img_ff, dim = -1) {

  rows <- dim(img_ff)[1]    
  cols <- dim(img_ff)[2]    

  swap_up_down <- function(img_ff) {
    rows_half <- ceiling(rows/2)
    return(rbind(img_ff[((rows_half+1):rows), (1:cols)], img_ff[(1:rows_half), (1:cols)]))
  }

  swap_left_right <- function(img_ff) {
    cols_half <- ceiling(cols/2)
    return(cbind(img_ff[1:rows, ((cols_half+1):cols)], img_ff[1:rows, 1:cols_half]))
  }

  if (dim == -1) {
    img_ff <- swap_up_down(img_ff)
    return(swap_left_right(img_ff))
  }
  else if (dim == 1) {
    return(swap_up_down(img_ff))
  }
  else if (dim == 2) {
    return(swap_left_right(img_ff))
  }
  else {
    stop("Invalid dimension parameter")
  }
}

ifftshift <- function(img_ff, dim = -1) {

  rows <- dim(img_ff)[1]    
  cols <- dim(img_ff)[2]    

  swap_up_down <- function(img_ff) {
    rows_half <- floor(rows/2)
    return(rbind(img_ff[((rows_half+1):rows), (1:cols)], img_ff[(1:rows_half), (1:cols)]))
  }

  swap_left_right <- function(img_ff) {
    cols_half <- floor(cols/2)
    return(cbind(img_ff[1:rows, ((cols_half+1):cols)], img_ff[1:rows, 1:cols_half]))
  }

  if (dim == -1) {
    img_ff <- swap_left_right(img_ff)
    return(swap_up_down(img_ff))
  }
  else if (dim == 1) {
    return(swap_up_down(img_ff))
  }
  else if (dim == 2) {
    return(swap_left_right(img_ff))
  }
  else {
    stop("Invalid dimension parameter")
  }
}
###################################################
###################################################
# FFT SHIFT

fft_shift <- fftshift(magnitude[,,1])

par(mfrow=c(1,2))

contour(magnitude[,,1])
contour(fft_shift)

filled.contour(fft_shift)

par(mfrow=c(1,2))

#az
plot(ts(fft_shift[,dim(fft_shift)[2]]))
#el
plot(ts(fft_shift[dim(fft_shift)[2],]))



x <- seq(0,dim(fft_shift)[1],1)  
y <- seq(0,dim(fft_shift)[2],1)
z <- fft_shift

#open3d(windowRect=c(10,10,700,700))
#bg3d("slategray")
#material3d(col = "blue")
#persp3d(z)
persp3D(z = fft_shift)
		
		
		
		

