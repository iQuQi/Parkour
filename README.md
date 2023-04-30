# 💬 프로젝트 소개

**모션 매칭**은 최근 게임 캐릭터 애니메이션을 위해 가장 많이 사용 되는 기법이다.

기존에는 **모션 그래프**를 이용한 방식이 사용되었으나, 동작이 다양해짐에 따라 그래프가 복잡해지고 많은 파라미터가 필요해지게 되는 등의 문제가 발생하였다.

이러한 이유로 모션 매칭 기법이 주목받기 시작했다. 모션 매칭은 **언제든지 다른 애니메이션으로 이동할 수 있으며, 더 높은 반응성을 가진다는 특징**이 있다.

<details>
<summary>Motion Graph</summary>
<div markdown="1">
![모션그래프](/uploads/c080b8bf41c1177de874fe833ed7c044/Untitled__2_.png)

✏️ **포즈와 속도가 어느정도 근접하게 유사해지면 전이점(transition point)에 다다름**

✏️ 이때 가능성 있는 것 중에서 어디로 갈지 결정

✏️ 다양한 문제점이 존재 → 자동으로 그래프를 구성하는 과정에서 제어와 유지가 어려움

- 스테이트 머신의 계층이 많아지고 확장됨에 따라 그래프가 복잡해짐
- 각 스테이트의 이동을 결정하거나 혼합하는 과정에서 많은 파리미터가 필요해짐
- 동작의 네이밍과 위치 선정이 쉽지 않음
- 루프 구현 시, 시작 동작과 끝 동작의 시점과 루프 변환 시점 선정이 어려움

</div>
</details>


<details>
<summary>Motion Matching의 전체 흐름</summary>
<div markdown="1">
![모션매칭](/uploads/a9fd4782328fa2bbca295edd176c9d2d/Untitled__3_.png)

### 포즈 DB 생성

1. 캐릭터에 애니메이션을 적용하기 위해서 Pose DB를 생성 
2. Pose 정보는 각 bone에 대한 Location과 Rotation 값 등을 포함

### 피쳐 DB 생성

1. 최상의 전환을 찾기 위해 Feature라는 데이터가 필요 
2. 이는 전환에 적합한 일치 항목을 찾기 위해 모션 일치 알고리즘에서 사용하는 애니메이션 데이터에서 추출된 속성
3. Feature는 위치나 속도와 같은 포즈 정보와, 궤적 정보를 포함
    - 포즈 정보: Pose DB 정보를 수정해서 사용
    - 궤적 정보: 현재 프레임으로부터 일정한 시간 간격으로 미래 위치를 계산

### 사용자 입력

1. 업데이트 시점이 되면 현재 포즈의 특징과 미래 궤적 예측 정보를 합해서 **쿼리 벡터** 생성
2. 쿼리 벡터는 포즈 정보와 궤적 정보로 이뤄져있으므로 피쳐와 동일한 구조

### 모션 매칭 알고리즘

1. KD-Tree 알고리즘을 통해 피쳐 DB에서 현재 쿼리 벡터와 가장 유사한 피쳐 검색
2. 즉, 현재 상황과 우리가 만들고자하는 모습을 잘 묘사하는 피쳐를 찾는 것
3. 매칭된 피쳐에 대응되는 포즈DB의 인덱스를 추출

### 애니메이션 교체

1. 새로운 애니메이션으로 교체해야하는 경우, 새로 매칭된 인덱스의 포즈로 변환
2. 그렇지 않은 경우, 포즈DB에서 인덱스를 하나씩 늘려가며 현재 애니메이션을 연속 재생
</div>
</details>

<br><br>

# 🎞️ 개발 프로세스

![image](https://user-images.githubusercontent.com/69742775/235349438-d5fcb6aa-d1aa-446c-bdd8-0dd5a115aa21.png)

<br><br>

# 🏆 프로젝트 결과
발표 슬라이드: [[제출]Motion Matching.pptx](https://github.com/iQuQi/Parkour/files/11361142/Motion.Matching.pptx)

결과 영상 링크: https://www.youtube.com/watch?v=_JCTBsr2cpY

[![Parkrour](http://img.youtube.com/vi/_JCTBsr2cpY/0.jpg)](https://youtu.be/_JCTBsr2cpY)



<br><br>

# 👩🏻‍💻 프로그램 코드
[노션](https://acidic-roundworm-3d2.notion.site/Motion-Matching-b86eddb3dd22469d9c09350fcfdaf75d)에서 확인해주세요 😀
