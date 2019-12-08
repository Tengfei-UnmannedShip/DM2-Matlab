module nonlinear
contains
!!!!!Define the Main Function
subroutine fun(dimen,mainf,x,old,fixnew,fixold)
implicit none
integer i,j,dimen
real,allocatable::mainf(:),tempf(:)
real old(3),x(3),fixold(3,3),fixnew(3,3)
allocate(mainf(dimen))
allocate(tempf(dimen))
do i=1,dimen
  tempf(i)=0.0
  do j=1,3
     tempf(i)=tempf(i)+(x(j)-fixnew(i,j))**2-(old(j)-fixold(i,j))**2
  end do
  mainf(i)=tempf(i)
end do
deallocate(tempf)
end subroutine fun

!!!!!Define the Derivate Function also called Jacobi Matrix F
subroutine jac(dimen,derif,x,fixnew)
implicit none
integer i,j,dimen
real,allocatable::derif(:,:)
real x(3),fixnew(3,3)
allocate(derif(dimen,dimen))  !!! normally it's a square-matrix
do i=1,dimen
   do j=1,dimen
      derif(i,j)=2*(x(j)-fixnew(i,j))
   end do
end do
end subroutine jac

!!!!!Calculation the Linear Function of the above functions
subroutine linear(derif,mainf,dx,dimen)
implicit none
integer i,j,k,max_line,dimen
real maxv,temp_x
real,allocatable::mainf(:),derif(:,:),ab(:,:),dx(:),temp(:,:)
!allocate(mainf(dimen))
!allocate(derif(dimen,dimen)) !!! normally it's a square-matrix
allocate(ab(dimen,dimen+1))
allocate(temp(dimen,dimen+1))
allocate(dx(dimen))

ab(1:dimen,1:dimen)=derif
ab(1:dimen,dimen+1)=-mainf
!!!elimination
do k=1,dimen-1     !!!search the max value of every column
   temp=0.0
   maxv=abs(ab(k,k))
   max_line=k
   do i=k+1,dimen
      if(abs(ab(i,k))>maxv)then
        maxv=abs(ab(i,k))
        max_line=i
      end if
   end do
   do j=k,dimen+1   !!!exchange the valume between max&current line
      temp(k,j)=ab(k,j)
      ab(k,j)=ab(max_line,j)
      ab(max_line,j)=temp(k,j)
   end do
   do i=k+1,dimen   !!!eliminate the x-coefficient gradually
      temp(i,k)=ab(i,k)/ab(k,k)
      do j=k+1,dimen+1
         ab(i,j)=ab(i,j)-temp(i,k)*ab(k,j)
      end do
   end do
end do
dx(dimen)=ab(dimen,dimen+1)/ab(dimen,dimen) !!!lead results
do k=dimen-1,1,-1
   temp_x=0.0
   do j=dimen,k+1,-1
      temp_x=temp_x+ab(k,j)*dx(j)
   end do
   dx(k)=(ab(k,dimen+1)-temp_x)/ab(k,k)
end do
!deallocate(ab)
deallocate(temp)
end subroutine linear
end module

!!!!!Main Program
program main
use nonlinear
implicit none
integer i,j,itera,dimen
real tole,temp_dx,value_dx
real old(3),x(3),fixold(3,3),fixnew(3,3) !!!new(3) is x(dimen)
real,allocatable::mainf(:),derif(:,:),dx(:)
data (old(i),i=1,3) / 4.843, 6.223, 4.973/ !which atomic you wanted
data (x(i),i=1,3)   /-1.100, 0.100,-1.900/ !NOT use ZERO
data ((fixold(i,j),j=1,3),i=1,3) / 5.943, 6.111, 6.883, 5.909, 6.052, 5.3370, 5.861, 5.0680, 7.4820/ !N,C,O
data ((fixnew(i,j),j=1,3),i=1,3) / 0.000, 0.000, 0.000, 0.000, 0.000,-1.5475, 0.000,-1.0692, 0.5568/ !N,C,O
dimen=3
itera=300   !!! The max iteration
tole=1.0D-5 !!! The tolerance
!allocate(mainf(dimen))
!allocate(derif(dimen))
!allocate(dx(dimen))
do i=1,itera
write(*,*)"###############",i,"###############"
   call fun(dimen,mainf,x,old,fixnew,fixold)  !!! For mainf
   call jac(dimen,derif,x,fixnew)             !!! For derif
   call linear(derif,mainf,dx,dimen)          !!! For dx
   do j=1,dimen
      x(j)=x(j)+dx(j)
   end do
   temp_dx=0.0
   do j=1,dimen
     temp_dx=temp_dx+dx(j)**2
   end do
   value_dx=sqrt(temp_dx)
   if(value_dx.le.tole)then
     write(*,*)"The results converge"
     write(*,*)x(:)
     exit
   end if
   if(i.eq.itera)then
     write(*,*)"Beyond the Max Iteration"
     exit
   end if
deallocate(mainf)
deallocate(derif)
deallocate(dx)
end do
end program main

