Trajectory fondue = drive.trajectoryBuilder(myPose)
                    //.back(20)
                    .lineTo(new Vector2d(-10, -50))
                    .addDisplacementMarker(()->{

                        LSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        LSlides.setTargetPosition(2400);
                        LSlides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        LSlides.setPower(0.2);
                        SlidesAngle.setPower(-0.4);
                        while(LSlides.isBusy()){
                            idle();

                        }
                        LSlides.setPower(0.0);
                        Bin.setPosition(1.0);
                        sleep(2000);
                        Bin.setPosition(0.5);
                        LSlides.setTargetPosition(0);
                        LSlides.setPower(-0.2);
                        while(LSlides.isBusy()){
                            idle();
                            SlidesAngle.setPower(-0.1);
                        }
                        LSlides.setPower(0);
                        SlidesAngle.setPower(0);



                    })
                    .build();

            //duck
            Trajectory duck = drive.trajectoryBuilder(fondue.end())
                    .lineTo(new Vector2d(34,4))
                    .addDisplacementMarker(()->{
                        ElapsedTime expo = new ElapsedTime(0);


                            expo.reset();
                            while(expo.time()<3.0){
                                fortuneIII = Math.pow(expo.time(),0.95);

                                Wheel.setPower(fortuneIII*-1);
                            }


                            Wheel.setPower(0);




            })
                    .build();
