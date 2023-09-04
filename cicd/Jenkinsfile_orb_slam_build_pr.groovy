node { 

    stage('Checkout') {
        echo "Checkout commit:  ${ghprbActualCommit}"
        def checkoutResult = checkout scm
        echo "Building CI with SCM: ${checkoutResult}"
    }

    stage('Clean previous docker image and container') { 
        sh "sudo docker rm -f slam"
        sh "sudo docker image rm -f slam"
    }

    stage ('Build and run docker') {
        sh "sudo chmod 777 ${env.WORKSPACE}/cicd/build_slam_in_docker.sh"
        sh "sudo docker build -f ${env.WORKSPACE}/Utils/Docker/Dockerfile -t slam . "
        sh "sudo docker run --rm --name slam  -v ${env.WORKSPACE}:/home/android/ORB_SLAM3  -w /home/android/ORB_SLAM3  slam cicd/build_slam_in_docker.sh" 
    }
}


 