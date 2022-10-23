using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using System.Threading;

public class MotionMatcher : MonoBehaviour
{
    // 초기화 안함..?
    public MotionsData motionsData;  // 깃헙에 정의된 클래스: public MotionData[] motionDataList;
    public MotionMatcherSettings motionMatcherSettings; // 깃헙에 정의된 클래스
    public MotionCostFactorSettings motionCostFactorSettings; // 깃헙에 정의된 클래스

    // 왜 필요?
    private PlayerInput playerInput; // 깃헙에 정의된 클래스

    // 초기화 
    public int bestMotionFrameIndex = -1;
    public string bestMotionName = "HumanoidIdle";
    private bool bCrouch = false;
    private float lastVelocity = 0f;

    // Start에서 초기화
    public MotionFrameData currentMotionFrameData; // 깃헙에 정의된 클래스
    private Transform motionOwner;
    private float currentComputeTime;

    // 디버깅 관련
    private string textContent = ""; 
    public Text costText;
    public MotionDebugData bestMotionDebugData = null; // 깃헙에 정의된 클래스
    public List<MotionDebugData> motionDebugDataList; 


    // 사용되는 곳 없음
    private Thread motionThread; 
    private bool bComputeMotionSnapShot = false;


    void Start()
    {
        currentMotionFrameData = new MotionFrameData();
        motionOwner = this.transform;
        currentComputeTime = motionMatcherSettings.ComputeMotionsBestCostGap;
        motionDebugDataList = new List<MotionDebugData>();

        // HumanoidCrouchIdle. 애니메이션에 문제가 있어 임시로 처리할 수밖에 없음
        // 모션 리스트에서 모션 꺼내기 => 모션에서 프레임 리스트 꺼내기 => 리스트에서 프레임 꺼내기 => 프레임에서 궤적정보 리스트 꺼내기 => 리스트에서 궤적 정보 꺼내기
        for (int i = 0; i < motionsData.motionDataList.Length; i++)
        {
            MotionData motionData = motionsData.motionDataList[i];
            if (motionData.motionName.Contains("HumanoidCrouchIdle")) // 임시 코드 인가봄
            {
                for (int j = 0; j < motionData.motionFrameDataList.Length; j++)
                {
                    MotionFrameData motionFrameData = motionData.motionFrameDataList[j];
                    for (int k = 0; k < motionFrameData.motionTrajectoryDataList.Length; k++)
                    {
                        MotionTrajectoryData motionTrajectoryData = motionFrameData.motionTrajectoryDataList[k];

                        // 꺼내온 궤적데이터의 로컬 포지션 값을 (0,0,0) 으로 초기화
                        motionTrajectoryData.localPosition = Vector3.zero;
                    }
                }
            }
        }
    }

    // 현재 시간 계산 함수
    private void Update()
    {
        currentComputeTime += Time.deltaTime;
    }

    // 상위 클래스에서 호출하는 메인 함수
    public string AcquireMatchedMotion(PlayerInput playerInput, string motionName, float normalizedTime)
    {
        //playerInput.crouch = true;
        //playerInput.direction = Vector3.forward;
        //playerInput.velocity = 0.65f;


        // 플레이어 입력정보 꺼내기
        float velocity = playerInput.velocity;
        Vector3 direction = playerInput.direction;
        float acceleration = playerInput.acceleration;
        float brake = playerInput.brake;
        bool crouch = playerInput.crouch;


        //
        if (currentComputeTime >= motionMatcherSettings.ComputeMotionsBestCostGap)
        {
            // 현재 시간 초기화
            currentComputeTime = 0;
    
            // 디버깅 관련 코드 - 0
            motionDebugDataList.Clear();

            // MotionMainEntryType: 깃헙에 정의된 Enum 타입 ( none,stand,crouch,prone,)
            MotionMainEntryType motionMainEntryType = MotionMainEntryType.none;

            // 웅크리기 변수 업데이트
            if (bCrouch != crouch)
            {
                bCrouch = crouch;
                motionMainEntryType = crouch ? MotionMainEntryType.crouch : MotionMainEntryType.stand;
            }
            
            // 속도 변수값 저장해두기
            lastVelocity = velocity;


            CapturePlayingMotionSnapShot(playerInput, motionName, normalizedTime, motionMainEntryType);

            // 디버깅 코드 - 1
            if (motionMatcherSettings.EnableDebugText)
            {
                AddDebugContent("Cost", true);
            }

            // 비용 계산 함수 호출
            ComputeMotionsBestCost();

            // 디버깅 코드 - 2
            costText.text = textContent;
            //Debug.LogFormat("AcquireMatchedMotion velocity {0} MotionName {1} direction {2}", velocity, MotionName, direction);
        }

        return bestMotionName;
    }


    // 호출하는 곳 없음
    // Baking: 모든 프레임에 키 프레임을 추가하는 과정으로, 제약 조건, 공간 전환 또는 시뮬레이션을 키 프레임으로 변환하는 것과 같은 다른 목적으로도 사용할 수 있음
    private float AcquireBakedMotionVelocity(string motionName, float normalizedTime, MotionMainEntryType motionMainEntryType)
    {
        MotionFrameData motionFrameData = AcquireBakedMotionFrameData(motionName, normalizedTime, motionMainEntryType);
        if (motionFrameData != null)
        {
            return motionFrameData.velocity;
        }
        return 0;
    }

    private MotionFrameData AcquireBakedMotionFrameData(string motionName, float normalizedTime, MotionMainEntryType motionMainEntryType)
    {
        MotionFrameData motionFrameData = null;
        for (int i = 0; i < motionsData.motionDataList.Length; i++)
        {
            MotionData motionData = motionsData.motionDataList[i];
            if (motionMainEntryType != MotionMainEntryType.none)
            {
                if (motionData.motionName.IndexOf(motionMatcherSettings.StandTag) >= 0 && 
                    motionMainEntryType == MotionMainEntryType.stand)
                {
                    motionFrameData = motionData.motionFrameDataList[0];
                    break;
                }
                if (motionData.motionName.IndexOf(motionMatcherSettings.CrouchTag) >= 0 &&
                    motionMainEntryType == MotionMainEntryType.crouch)
                {
                    motionFrameData = motionData.motionFrameDataList[0];
                    break;
                }
            }
            else if (motionData.motionName == motionName)
            {
                int frame = Mathf.FloorToInt(motionData.motionFrameDataList.Length * normalizedTime);
                motionFrameData = motionData.motionFrameDataList[frame];
            }
        }
        return motionFrameData;
    }

    private void CapturePlayingMotionSnapShot(PlayerInput playerInput, string motionName, float normalizedTime, MotionMainEntryType motionMainEntryType)
    {
        float velocity = playerInput.velocity;
        float angularVelocity = playerInput.angularVelocity;
        Vector3 direction = playerInput.direction;
        float acceleration = playerInput.acceleration;
        float brake = playerInput.brake;
  
        MotionFrameData bakedMotionFrameData = AcquireBakedMotionFrameData(motionName, normalizedTime, motionMainEntryType);
        currentMotionFrameData.velocity = velocity;
        currentMotionFrameData.motionBoneDataList = bakedMotionFrameData.motionBoneDataList;
        currentMotionFrameData.motionTrajectoryDataList = new MotionTrajectoryData[motionMatcherSettings.predictionTrajectoryTimeList.Length];

        for (int i = 0; i < motionMatcherSettings.predictionTrajectoryTimeList.Length; i++)
        {
            float predictionTrajectoryTime = motionMatcherSettings.predictionTrajectoryTimeList[i];
            currentMotionFrameData.motionTrajectoryDataList[i] = new MotionTrajectoryData();
            MotionTrajectoryData motionTrajectoryData = currentMotionFrameData.motionTrajectoryDataList[i];
            motionTrajectoryData.localPosition = velocity * direction * predictionTrajectoryTime;
            motionTrajectoryData.velocity = velocity * direction;
            if (Mathf.Abs(playerInput.angularVelocity) > 0)
            {
                motionTrajectoryData.direction = Quaternion.Euler(0, playerInput.angularVelocity * predictionTrajectoryTime, 0) * Vector3.forward;
            }
        }
    }

    private void ComputeMotionsBestCost()
    {
        float bestMotionCost = float.MaxValue;
        bestMotionName = "";
        bestMotionFrameIndex = 0;
        for (int i = 0; i < motionsData.motionDataList.Length; i++)
        {
            MotionData motionData = motionsData.motionDataList[i];
            if (motionMatcherSettings.EnableDebugText)
            {
                AddDebugContent("motion: " + motionData.motionName);
            }

            for (int j = 0; j < motionData.motionFrameDataList.Length; j++)
            {
                MotionDebugData motionDebugData = new MotionDebugData();
                float motionCost = 0f;
                float bonesCost = 0f;
                float bonePosCost = 0f;
                float boneRotCost = 0f;

                float trajectoryPosCost = 0f;
                float trajectoryVelCost = 0f;
                float trajectoryDirCost = 0f;
                float trajectorysCost = 0f;

                float rootMotionCost = 0f;

                MotionFrameData motionFrameData = motionData.motionFrameDataList[j];

                for (int k = 0; k < motionFrameData.motionBoneDataList.Length; k++)
                {
                    MotionBoneData motionBoneData = motionFrameData.motionBoneDataList[k];
                    MotionBoneData currentMotionBoneData = currentMotionFrameData.motionBoneDataList[k];
                    float BonePosCost = Vector3.SqrMagnitude(motionBoneData.localPosition - currentMotionBoneData.localPosition);
                    Quaternion BonePosError = Quaternion.Inverse(motionBoneData.localRotation) * currentMotionBoneData.localRotation;
                    float BoneRotCost = Mathf.Abs(BonePosError.x) + Mathf.Abs(BonePosError.y) + Mathf.Abs(BonePosError.z) + (1 - Mathf.Abs(BonePosError.w));
                    //float BoneVelocityCost = Vector3.SqrMagnitude(motionBoneData.velocity - currentMotionBoneData.velocity);
                    bonePosCost += BonePosCost * motionCostFactorSettings.bonePosFactor;
                    boneRotCost += BoneRotCost * motionCostFactorSettings.boneRotFactor/* + BoneVelocityCost * motionCostFactorSettings.boneVelFactor*/;
                    //AddDebugContent("BonePosCost: " + BonePosCost);
                    //AddDebugContent("BoneRotCost: " + BoneRotCost);
                    //AddDebugContent("BoneVelocityCost: " + BoneVelocityCost);
                }

                bonesCost = bonePosCost + boneRotCost;
                motionDebugData.bonePosCost = bonePosCost;
                motionDebugData.boneRotCost = boneRotCost;
                motionDebugData.bonesCost = bonesCost;

                if (motionMatcherSettings.EnableDebugText)
                {
                    AddDebugContent("bonesTotalCost: " + bonesCost);
                }

                for (int l = 0; l < motionFrameData.motionTrajectoryDataList.Length; l++)
                {
                    MotionTrajectoryData motionTrajectoryData = motionFrameData.motionTrajectoryDataList[l];
                    MotionTrajectoryData currentMotionTrajectoryData = currentMotionFrameData.motionTrajectoryDataList[l];

                    trajectoryPosCost += Vector3.SqrMagnitude(motionTrajectoryData.localPosition - currentMotionTrajectoryData.localPosition) * motionCostFactorSettings.predictionTrajectoryPosFactor;
                    //trajectoryVelCost += Vector3.SqrMagnitude(motionTrajectoryData.velocity - currentMotionTrajectoryData.velocity) * motionCostFactorSettings.predictionTrajectoryVelFactor;
                    trajectoryDirCost += Vector3.Dot(motionTrajectoryData.direction, currentMotionTrajectoryData.direction) * motionCostFactorSettings.predictionTrajectoryDirFactor;
                    //AddDebugContent("trajectoryPosCost: " + trajectoryPosCost);
                    //AddDebugContent("trajectoryVelCost: " + trajectoryVelCost);
                    //AddDebugContent("trajectoryDirCost: " + trajectoryDirCost);
                }

                trajectorysCost = trajectoryPosCost + trajectoryVelCost + trajectoryDirCost;
                motionDebugData.trajectoryPosCost = trajectoryPosCost;
                motionDebugData.trajectoryVelCost = trajectoryVelCost;
                motionDebugData.trajectoryDirCost = trajectoryDirCost;
                motionDebugData.trajectorysCost = trajectorysCost;

                if (motionMatcherSettings.EnableDebugText)
                {
                    AddDebugContent("trajectorysToatalCost: " + trajectorysCost);
                }

                rootMotionCost = Mathf.Abs(motionFrameData.velocity - currentMotionFrameData.velocity) * motionCostFactorSettings.rootMotionVelFactor;
                motionDebugData.rootMotionCost = rootMotionCost;
                if (motionMatcherSettings.EnableDebugText)
                {
                    AddDebugContent("rootMotionCost: " + rootMotionCost);
                }

                motionCost = bonesCost + trajectorysCost + rootMotionCost;
                motionDebugData.motionCost = motionCost;

                if (motionMatcherSettings.EnableDebugText)
                {
                    AddDebugContent("motionTotalCost: " + motionCost);
                }

                //Debug.LogFormat("ComputeMotionsBestCost motionName {0} motionCost {1} ", motionData.motionName, motionCost);

                if (bestMotionCost > motionCost)
                {
                    bestMotionCost = motionCost;
                    bestMotionFrameIndex = j;
                    bestMotionName = motionData.motionName;
                    bestMotionDebugData = motionDebugData;

                    motionDebugData.motionName = motionData.motionName;
                    motionDebugData.motionFrameIndex = j;

                    motionDebugDataList.Add(motionDebugData);
                }
            }
        }
    }


    // 디버깅 코드 함수
    private void AddDebugContent(string content, bool bClear = false)
    {
        if (bClear)
        {
            textContent = "";
        }
        textContent += content;
        textContent += "\n";
    }
}