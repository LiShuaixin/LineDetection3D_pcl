#ifndef _PLANE_DETECTION_H_
#define _PLANE_DETECTION_H_

#include "CommonFunctions.h"

// template<typename CoordType>
// struct Votes
// {
//     CoordType m_dMax;
//     CoordType m_dMin;
//     CoordType m_dInterval; 
//     unsigned m_nInterval; 
//     CoordType* m_pVotes;
//     CoordType m_nMaxVote; 
//     unsigned m_nTotalVotedVoxel; 
//     unsigned m_nTotalVotedVoxelPoints;
//     unsigned m_nValidVoxel;
//     CoordType m_dRMSThreshold_voted; 
//     CoordType m_dMaxRMSVotesCorrespondes;
//     CoordType m_dAlphaThreshold_Top; 
//     CoordType m_dAlphaThreshold_Bottom;
//     CoordType m_dAlphaThreshold;
//     CoordType m_dVoxelWidthThreshold; 
//     CoordType m_dMaxRMS;
//     
//     Votes(): m_dMax(0.1),
// 	     m_dMin(0.0),
// 	     m_dInterval(0.0001),
// 	     m_nInterval(1000),
// 	     m_pVotes(NULL),
// 	     m_nMaxVote(0),
// 	     m_nTotalVotedVoxel(0),
// 	     m_nTotalVotedVoxelPoints(0),
// 	     m_dRMSThreshold_voted(-1.0),
// 	     m_dMaxRMSVotesCorrespondes(0.0),
// 	     m_dAlphaThreshold_Top(1.0),
// 	     m_dAlphaThreshold_Bottom(0.5),
// 	     m_dAlphaThreshold(0.5),
// 	     m_dVoxelWidthThreshold(0.0),
// 	     m_dMaxRMS(0.0),
// 	     m_nValidVoxel(0) {}
// 	    
//     virtual Votes() {destroy();}
//     
//     virtual void initialize(CoordType dMax = 0.1, CoordType dMin = 0.0, CoordType dInterval = 0.0001)
//     {
// 	if (m_pVotes) return;
// 	m_nMaxVote = 0.0;
// 	m_nTotalVotedVoxel = 0;
// 	m_nTotalVotedVoxelPoints = 0;
// 	m_dRMSThreshold_voted = -1.0;
// 	m_dMaxRMSVotesCorrespondes = 0.0;
// 	m_dAlphaThreshold_Top = 1.0;
// 	m_dAlphaThreshold_Bottom = 0.5;
// 	m_dAlphaThreshold = 0.99;
// 	m_dVoxelWidthThreshold = 0.0;
// 	m_nValidVoxel = 0;
// 	m_dMaxRMS = 0.0;
// 	m_dMax = dMax;
// 	m_dMin = dMin;
// 	m_dInterval = dInterval;
// 	m_nInterval = (unsigned)((m_dMax-m_dMin)/m_dInterval);
// 	m_pVotes = new CoordType [m_nInterval];
// 	memset(m_pVotes, 0, sizeof(CoordType)*m_nInterval);
//     }
//     
//     virtual void initialize(CoordType dAlphaThreshold_Current, CoordType dAlphaThreshold_Top = 1.0, 
// 			    CoordType dAlphaThreshold_Bottom = 0.5, CoordType dVoxelWidthThreshold = 0.0, 
// 			    CoordType dRMSThreshold_LastVoted = -1.0)
//     {
// 	initialize();
// 	m_dRMSThreshold_voted = dRMSThreshold_LastVoted;
// 	m_dAlphaThreshold_Top = dAlphaThreshold_Top;
// 	m_dAlphaThreshold_Bottom = dAlphaThreshold_Bottom;
// 	m_dAlphaThreshold = dAlphaThreshold_Current;
// 	m_dVoxelWidthThreshold = dVoxelWidthThreshold;
//     }
// 	
//     virtual void ReSetThreshold(CoordType dAlphaThreshold_Current, CoordType dAlphaThreshold_Top = 1.0, 
// 				CoordType dAlphaThreshold_Bottom = 0.5, CoordType dVoxelWidthThreshold = 0.0, 
// 				CoordType dRMSThreshold_LastVoted = -1.0)
//     {
// 	m_dRMSThreshold_voted = dRMSThreshold_LastVoted;
// 	m_dAlphaThreshold_Top = dAlphaThreshold_Top;
// 	m_dAlphaThreshold_Bottom = dAlphaThreshold_Bottom;
// 	m_dAlphaThreshold = dAlphaThreshold_Current;
// 	m_dVoxelWidthThreshold = dVoxelWidthThreshold;
//     }
// 	
//     void vote(CoordType votingObj, CoordType voxelSize, unsigned nPoints, CoordType nCurrentVotes = 1.0)
//     {
// 	if (!m_pVotes) return;
// 	unsigned num = (unsigned)((votingObj - m_dMin)/m_dInterval); 
// 	num = min(num, (m_nInterval - 1)); 
// 	m_pVotes[num] += nCurrentVotes;
// 	m_nTotalVotedVoxelPoints += nPoints;
// 	m_nTotalVotedVoxel += 1;
// 	m_dMaxRMS = max(m_dMaxRMS, votingObj);
//     }
// 	void getVotingResults()
// 	{
// 		if (!m_pVotes || !m_nTotalVotedVoxel) return;
// 		if (m_nTotalVotedVoxel < (m_nValidVoxel * 0.01) || m_nValidVoxel < 3) m_dRMSThreshold_voted = m_dMaxRMSVotesCorrespondes = m_dMaxRMS;
// 		else
// 		{
// 			/***********************��ֵ̽��ͳ��ƽ�����صķֲ�*********************/
// 			m_nMaxVote = m_pVotes[1];
// 			m_dRMSThreshold_voted = m_dMin + m_dInterval * (1.0 + 0.5);
// 			for (unsigned i = 1; i < m_nInterval - 1; i ++)
// 			{
// 				if (m_nMaxVote < m_pVotes[i])
// 				{
// 					m_nMaxVote = m_pVotes[i];
// 					m_dMaxRMSVotesCorrespondes = m_dMin + m_dInterval * ((CoordType)i + 0.5);
// 					m_dRMSThreshold_voted = m_dMaxRMSVotesCorrespondes;
// 				}
// 			}
// 			for (unsigned i = (unsigned)((m_dRMSThreshold_voted - m_dMin) / m_dInterval - 0.5); i < m_nInterval - 1; i ++)
// 			{
// 				if (m_pVotes[i] <= min(3.0, (m_nMaxVote * 0.01)))
// 				{
// 					m_dRMSThreshold_voted = m_dMin + m_dInterval * ((CoordType)i + 0.5);
// 					break;
// 				}
// 			}
// 		}
// 
// 		m_dRMSThreshold_voted = max(m_dRMSThreshold_voted, 1E-06);
// 		m_dVoxelWidthThreshold = (((m_dRMSThreshold_voted * 2.0) * 10.0) / sqrt(3.0)) * 2.0;
// 
// 		//����ͶƱ�ռ�
// 		if ((m_dMaxRMSVotesCorrespondes / m_dInterval) > 50.0)/* || m_nMaxVote < max(3.0, (m_nTotalVotedVoxel * 0.01))*/
// 		{
// 			m_dInterval *= 10.0; m_nInterval = (unsigned)((m_dMax-m_dMin)/m_dInterval); //ͶƱ������
// 			CoordType* pCurrentVotes = new CoordType [m_nInterval]; //ͶƱͳ�Ʊ�����Ʊ��
// 			memset(pCurrentVotes, 0, sizeof(CoordType)*m_nInterval); //����
// 			for (unsigned i = 0; i < m_nInterval; i ++) for (unsigned j = 0; j < 10; j ++) pCurrentVotes[i] += m_pVotes[i * 10 + j]; 
// 			delete [] m_pVotes;
// 			m_pVotes = pCurrentVotes;
// 			getVotingResults();
// 		}
// 	}
// 	virtual void destroy()
// 	{
// 		if (m_pVotes)
// 		{
// 			delete [] m_pVotes;
// 			m_pVotes = NULL;
// 		}
// 		m_nTotalVotedVoxel = 0;
// 		m_nTotalVotedVoxelPoints = 0;
// 	}
// 	void setZero()
// 	{
// 		memset(m_pVotes, 0, sizeof(unsigned) * m_nInterval);
// 		m_nTotalVotedVoxel = 0;
// 		m_nTotalVotedVoxelPoints = 0;
// 		m_nValidVoxel = 0;
// 		m_dMaxRMS = 0.0;
// 	}
// };
/*		
template<typename CoordType, typename PointType>
struct TreeCellFunctionParameter*/
// {
//     CoordType m_dOcTreeBoxWidth_inBlock; //�˲������Ĳ����ڵ��еĿ���
//     CoordType m_dVoxelGlobalHightThreshold; //������������ʶ������ʱ��ȫ�ָ߶���ֵ
//     CoordType m_dGrowedHightThreshold; //������������ʶ������ʱ�������߶���ֵ
//     CoordType m_dSearchRadius; //���������뾶
//     CoordType m_dNormalCosinCrossingAngleThreshold; //��ʸ�нǵ���ֵ�����ң�
//     CoordType m_dDistanceThreshold; //�������ֵ�����ң�
//     unsigned m_nThreshold; //�������ֵ
//     unsigned m_nExpelledPoints; //���ų��ĵ���
//     CoordType m_dAngleThreshold; //�Ƕ���ֵ
//     CoordType m_dVotingInterval; //���ʣ����
//     CoordType m_dPlaneProbability; //ƽ��ĸ���
//     void* m_pVoidPtr; //�洢�������͵�ָ��
//     std::vector<void*> m_vSeedPtr; //�洢�������е�����ָ��
//     std::vector<CoordType>* m_statisticalParameter; //
//     std::vector<PointType>* m_treeGrid; //�洢���ṹ�ڵ�İ�Χ��
//     Votes<CoordType>* m_Votes; 
//     unsigned m_nVoxel; //���ص��ܸ���
//     unsigned m_nValidVoxel; //�������������ظ���
//     uchar m_nMaxVoxelLevel; //���ص�������
//     unsigned m_nClass; //����ܸ���
//     std::vector<template_Struct_Voxel_Plane<CoordType, PointType>> m_vPlanes; //����ƽ��ṹ��Ķ���
//     std::vector<template_Struct_V_Shape<CoordType, PointType>> m_vVShapes; //v-Shapes�ṹ��Ķ���
//     std::vector<template_Struct_Poly_Shape<CoordType, PointType>> m_vPolyShapes; //����νṹ��Ķ���
//     LineSegmentType m_uLineSegmentType; //�߶���ȡ�����ͣ�0Ϊ�ṹ�ߣ�1Ϊ��Ե�ߣ�... ...
// 	union
// 	{
// 		struct
// 		{
// 			clock_t m_tStart;//���㿪ʼ��ʱ��㡪��time_Start
// 			clock_t m_tOG;//�˲����ʷֺ�ʱ����time_OctreeGeneration
// 			clock_t m_tPE;//ƽ����ȡ��ʱ����time_PlaneExtraction
// 			clock_t m_tLSE;//�߶���ȡ��ʱ����time_LineSegmentExtraction
// 			clock_t m_tRefine;//����Ż���ʱ����time_Refinement
// 		};
// 		clock_t m_time[5]; //ʱ��
// 	};
// 	CoordType m_dVShapeSigma;//�洢�߶ε�֧�����㵽֧������ľ���������洢�߶ε�ƽ�����Ŷ�
// 	PointType m_CloudOrigin; //��������ԭ��
// 	BOOL m_bGetPolyShape; //��ȡ����νṹ
// 	BOOL m_bGetSemanticLine; //��ȡ�����߶���
// 	TreeCellFunctionParameter():
// 		m_dOcTreeBoxWidth_inBlock(0), 
// 		m_dVoxelGlobalHightThreshold(0), 
// 		m_dGrowedHightThreshold(0),
// 		m_dSearchRadius(0),
// 		m_dNormalCosinCrossingAngleThreshold(0),
// 		m_dDistanceThreshold(0),
// 		m_nThreshold(0),
// 		m_nExpelledPoints(0),
// 		m_dAngleThreshold(cos(2.0 * atan(0.1))),
// 		m_dVotingInterval(0.01),
// 		m_dPlaneProbability(0.5),
// 		m_pVoidPtr(NULL),
// 		m_statisticalParameter(NULL),
// 		m_treeGrid(NULL),
// 		m_Votes(NULL),
// 		m_nVoxel(0),
// 		m_nValidVoxel(0),
// 		m_nMaxVoxelLevel(0),
// 		m_nClass(0),
// 		m_uLineSegmentType(NONE_LINE),
// 		m_dVShapeSigma(0),
// 		m_bGetPolyShape(FALSE),
// 		m_bGetSemanticLine(FALSE)
// 	{
// 	}
// 	virtual ~TreeCellFunctionParameter() {clear();}
// 	virtual void clear()
// 	{
// 		if (m_vSeedPtr.size())
// 		{
// 			m_vSeedPtr.clear();
// 			m_vSeedPtr.swap(std::vector<void*> ());
// 		}
// 		if (m_treeGrid)
// 		{
// 			m_treeGrid->clear();
// 			m_treeGrid->swap(std::vector<PointType> ());
// 			delete m_treeGrid;
// 			m_treeGrid = NULL;
// 		}
// 		if (m_statisticalParameter)
// 		{
// 			m_statisticalParameter->clear();
// 			m_statisticalParameter->swap(std::vector<CoordType> ());
// 			delete m_statisticalParameter;
// 			m_statisticalParameter = NULL;
// 		}				
// 		if (m_Votes)
// 		{
// 			m_Votes->destroy();
// 			delete m_Votes;
// 			m_Votes = NULL;
// 		}
// 		if (m_vPlanes.size())
// 		{
// 			for (unsigned i = 0; i < m_vPlanes.size(); i ++) m_vPlanes[i].clear();
// 			m_vPlanes.clear();
// 			m_vPlanes.swap(std::vector<template_Struct_Voxel_Plane<CoordType, PointType>> ());
// 		}
// 		if (m_vVShapes.size())
// 		{
// 			for (unsigned i = 0; i < m_vVShapes.size(); i ++) m_vVShapes[i].clear();
// 			m_vVShapes.clear();
// 			m_vVShapes.swap(std::vector<template_Struct_V_Shape<CoordType, PointType>> ());
// 		}
// 		if (m_vPolyShapes.size())
// 		{
// 			for (unsigned i = 0; i < m_vPolyShapes.size(); i ++) m_vPolyShapes[i].clear();
// 			m_vPolyShapes.clear();
// 			m_vPolyShapes.swap(std::vector<template_Struct_Poly_Shape<CoordType, PointType>> ());
// 		}
// 	}
// 	virtual void initializeVotes()
// 	{
// 	    if (m_Votes)
// 	    {
// 		m_Votes->destroy();
// 		m_Votes = NULL;
// 	    }
// 	    m_Votes = new Votes<PointCoordinateType>();
// 	    m_Votes->initialize();
// 	}
// 	virtual void initializeVotes(CoordType dAlphaThreshold_Current, CoordType dAlphaThreshold_Top = 1.0, CoordType dAlphaThreshold_Bottom = 0.5, CoordType dVoxelWidthThreshold = 0.0, CoordType dRMSThreshold_LastVoted = -1.0)
// 	{
// 		if (m_Votes)
// 		{
// 			m_Votes->destroy();
// 			m_Votes = NULL;
// 		}
// 		m_Votes = new Votes<PointCoordinateType>(); //�������ɷַ���ͶƱͳ�Ʊ���
// 		m_Votes->initialize(dAlphaThreshold_Current, dAlphaThreshold_Top, dAlphaThreshold_Bottom, dVoxelWidthThreshold, dRMSThreshold_LastVoted);
// 	}
// 	virtual void ReSetThreshold(CoordType dAlphaThreshold_Current, CoordType dAlphaThreshold_Top = 1.0, CoordType dAlphaThreshold_Bottom = 0.5, CoordType dVoxelWidthThreshold = 0.0, CoordType dRMSThreshold_LastVoted = -1.0)
// 	{
// 		m_Votes->ReSetThreshold(dAlphaThreshold_Current, dAlphaThreshold_Top, dAlphaThreshold_Bottom, dVoxelWidthThreshold, dRMSThreshold_LastVoted);
// 	}
// 	inline static bool AreaCompare(const template_Struct_Voxel_Plane<CoordType, PointType>&a, const template_Struct_Voxel_Plane<CoordType, PointType>&b) throw() {return a.m_dArea > b.m_dArea;};
// 	struct VShapeCluster
// 	{
// 		PointType Cluster;
// 		unsigned nQuantity;
// 		CoordType dTotalLength;
// 		BYTE m_RGB[3]; //�洢��ɫ
// 		VShapeCluster() : nQuantity(0), dTotalLength(0.0) {memset(m_RGB, 0, sizeof(BYTE) * 3);}
// 		VShapeCluster(PointType pt, CoordType dLength = 0.0) {Cluster = pt; nQuantity = 1; dTotalLength = dLength;}
// 		inline VShapeCluster& operator = (const VShapeCluster &tempCluser) {Cluster = tempCluser.Cluster; nQuantity = tempCluser.nQuantity; dTotalLength = tempCluser.dTotalLength; return *this;}
// 		inline static bool LengthCompare(const VShapeCluster a, const VShapeCluster b) throw() {return (a.dTotalLength > b.dTotalLength);};
// 		void setColor(BYTE rgb[3]){memcpy(m_RGB, rgb, sizeof(BYTE) * 3);}
// 	};
// 	void VShapes_KMeansCluster_byAmount(CoordType dCosinCrossAngleThreshold, unsigned nArrayThreshold)
// 	{
// 		unsigned nIterated = 0; //�����Ĵ���
// 		std::vector<VShapeCluster> vClusters; //�洢��̬�����������
// 		std::vector<VShapeCluster> vCenters; //�洢�ڸ����ƽ������
// 		while (true)
// 		{
// 			nIterated ++;
// 			for (unsigned i = 0; i < m_vVShapes.size(); i ++) //������������Ķ�̬����
// 			{
// 				if (m_vVShapes[i].m_lineDirection.norm2() < 1E-08) continue; 
// 				bool bClustered = false; 
// 				for (unsigned j = 0; j < vClusters.size(); j ++)
// 				{
// 					CoordType dCorssAngleCosin = fabs(m_vVShapes[i].m_lineDirection.dot(vClusters[j].Cluster));
// 					if (dCorssAngleCosin > dCosinCrossAngleThreshold)
// 					{
// 						bClustered = true; 
// 						vCenters[j].Cluster += m_vVShapes[i].m_lineDirection;
// 						vCenters[j].nQuantity ++; 
// 						vCenters[j].dTotalLength += m_vVShapes[i].m_lineLength; 
// 						break; 
// 					}
// 				}
// 				if (!bClustered && nIterated == 1) 
// 				{
// 					vClusters.push_back(VShapeCluster(m_vVShapes[i].m_lineDirection, m_vVShapes[i].m_lineLength));
// 					vCenters.push_back(VShapeCluster(m_vVShapes[i].m_lineDirection, m_vVShapes[i].m_lineLength));
// 				}
// 			}
// 			CoordType dNM = 2.0, dDM = 0.0; 
// 			for (unsigned i = 0; i < vCenters.size(); i ++)
// 			{
// 				if (vCenters[i].nQuantity < 1) //����������ֵ��ɾ����
// 				{
// 					vClusters[i] = vClusters[vClusters.size() - 1]; vClusters.pop_back();
// 					vCenters[i] = vCenters[vCenters.size() - 1]; vCenters.pop_back();
// 					i --; 
// 					continue; 
// 				}
// 				vCenters[i].dTotalLength /= vCenters[i].nQuantity;
// 				vCenters[i].Cluster /= vCenters[i].nQuantity;
// 				vCenters[i].Cluster.normalize();
// 				bool bSame = false; 
// 				for (unsigned j = 0; j < i; j ++)
// 				{
// 					CoordType dTempCrossAngleCosin = fabs(vCenters[i].Cluster.dot(vClusters[j].Cluster)); 
// 					if (dTempCrossAngleCosin > dCosinCrossAngleThreshold)
// 					{
// 						bSame = true; 
// 						break; 
// 					}
// 				}
// 				if (!bSame)
// 				{
// 					dNM = min(dNM, fabs(vCenters[i].Cluster.dot(vClusters[i].Cluster)));
// 					vClusters[i] = vCenters[i]; 
// 					vCenters[i].Cluster.x = 0.0;
// 					vCenters[i].Cluster.y = 0.0;
// 					vCenters[i].Cluster.z = 0.0;
// 					vCenters[i].nQuantity = 0;
// 					vCenters[i].dTotalLength = 0.0;
// 				}
// 				else
// 				{
// 					vClusters[i] = vClusters[vClusters.size() - 1]; vClusters.pop_back();
// 					vCenters[i] = vCenters[vCenters.size() - 1]; vCenters.pop_back();
// 					i --; 
// 				}
// 			}//for (unsigned i = 0; i < vClusters.size(); i ++)
// 			if (dNM > dCosinCrossAngleThreshold) break; //
// 		}//while(true)
// 		    vCenters.clear(); vCenters.swap(std::vector<VShapeCluster>());
// 		    std::sort(vClusters.begin(), vClusters.end(), VShapeCluster::LengthCompare);
// 		    for (unsigned i = 0; i < m_vVShapes.size(); i ++) //��������
// 		    {
// 			    bool bClustered = false; 
// 			    for (unsigned j = 0; j < nArrayThreshold; j ++)
// 			    {
// 				    CoordType dCorssAngleCosin = fabs(m_vVShapes[i].m_lineDirection.dot(vClusters[j].Cluster));
// 				    if (dCorssAngleCosin > dCosinCrossAngleThreshold)
// 				    {
// 					    bClustered = true; 
// 					    break; 
// 				    }
// 			    }
// 			    if (!bClustered)
// 			    {
// 				    m_vVShapes[i] = m_vVShapes[m_vVShapes.size() - 1];
// 				    m_vVShapes.pop_back();
// 				    i--;
// 			    }
// 		    }
// 		    vClusters.clear(); vClusters.swap(std::vector<VShapeCluster>());
// 	}
// 	void VShapes_KMeansCluster_byLength(CoordType dCosinCrossAngleThreshold, unsigned nArrayThreshold)
// 	{
// 		unsigned nIterated = 0; //�����Ĵ���
// 		std::vector<VShapeCluster> vClusters; //�洢��̬�����������
// 		std::vector<VShapeCluster> vCenters; //�洢�ڸ����ƽ������
// 		while (true)
// 		{
// 			nIterated ++;
// 			for (unsigned i = 0; i < m_vVShapes.size(); i ++) //������������Ķ�̬����
// 			{
// 				if (m_vVShapes[i].m_lineDirection.norm2() < 1E-08) continue; 
// 				bool bClustered = false; 
// 				for (unsigned j = 0; j < vClusters.size(); j ++)
// 				{
// 					CoordType dCorssAngleCosin = fabs(m_vVShapes[i].m_lineDirection.dot(vClusters[j].Cluster));
// 					if (dCorssAngleCosin > dCosinCrossAngleThreshold)
// 					{
// 						bClustered = true; 
// 						vCenters[j].Cluster += m_vVShapes[i].m_lineDirection;
// 						vCenters[j].nQuantity ++; 
// 						vCenters[j].dTotalLength += m_vVShapes[i].m_lineLength; 
// 						break; 
// 					}
// 				}
// 				if (!bClustered && nIterated == 1) 
// 				{
// 					vClusters.push_back(VShapeCluster(m_vVShapes[i].m_lineDirection, m_vVShapes[i].m_lineLength));
// 					vCenters.push_back(VShapeCluster(m_vVShapes[i].m_lineDirection, m_vVShapes[i].m_lineLength));
// 				}
// 			}
// 			CoordType dNM = 2.0, dDM = 0.0; 
// 			for (unsigned i = 0; i < vCenters.size(); i ++)
// 			{
// 				if (vCenters[i].nQuantity < 1) //����������ֵ��ɾ����
// 				{
// 					vClusters[i] = vClusters[vClusters.size() - 1]; vClusters.pop_back();
// 					vCenters[i] = vCenters[vCenters.size() - 1]; vCenters.pop_back();
// 					i --; 
// 					continue; 
// 				}
// 				vCenters[i].dTotalLength /= vCenters[i].nQuantity;
// 				vCenters[i].Cluster /= vCenters[i].nQuantity;
// 				vCenters[i].Cluster.normalize();
// 				bool bSame = false; 
// 				for (unsigned j = 0; j < i; j ++)
// 				{
// 					CoordType dTempCrossAngleCosin = fabs(vCenters[i].Cluster.dot(vClusters[j].Cluster)); 
// 					if (dTempCrossAngleCosin > dCosinCrossAngleThreshold)
// 					{
// 						bSame = true; 
// 						break; 
// 					}
// 				}
// 				if (!bSame)
// 				{
// 					dNM = min(dNM, fabs(vCenters[i].Cluster.dot(vClusters[i].Cluster)));
// 					vClusters[i] = vCenters[i]; 
// 					vCenters[i].Cluster.x = 0.0;
// 					vCenters[i].Cluster.y = 0.0;
// 					vCenters[i].Cluster.z = 0.0;
// 					vCenters[i].nQuantity = 0;
// 					vCenters[i].dTotalLength = 0.0;
// 				}
// 				else
// 				{
// 					vClusters[i] = vClusters[vClusters.size() - 1]; vClusters.pop_back();
// 					vCenters[i] = vCenters[vCenters.size() - 1]; vCenters.pop_back();
// 					i --; 
// 				}
// 			}//for (unsigned i = 0; i < vClusters.size(); i ++)
// 			if (dNM > dCosinCrossAngleThreshold) break; //
// 		}//while(true)
// 		vCenters.clear(); vCenters.swap(std::vector<VShapeCluster>());
// 		std::sort(vClusters.begin(), vClusters.end(), VShapeCluster::LengthCompare);
// 		for (unsigned i = 0; i < m_vVShapes.size(); i ++) //��������
// 		{
// 			bool bClustered = false; 
// 			for (unsigned j = 0; j < nArrayThreshold; j ++)
// 			{
// 				CoordType dCorssAngleCosin = fabs(m_vVShapes[i].m_lineDirection.dot(vClusters[j].Cluster));
// 				if (dCorssAngleCosin > dCosinCrossAngleThreshold)
// 				{
// 					bClustered = true; 
// 					break; 
// 				}
// 			}
// 			if (!bClustered)
// 			{
// 				m_vVShapes[i] = m_vVShapes[m_vVShapes.size() - 1];
// 				m_vVShapes.pop_back();
// 				i--;
// 			}
// 		}
// 		vClusters.clear(); vClusters.swap(std::vector<VShapeCluster>());
// 	}
// 	void VShapes_ClusterbyAmount(std::vector<template_Struct_V_Shape<CoordType, PointType>>&vVShapes, CoordType dCosinCrossAngleThreshold, unsigned nArrayThreshold)
// 	{
// 		for (unsigned i = 0; i < vVShapes.size(); i ++) //����ɾ���ǿɿ��߶�
// 		{
// 			if (vVShapes[i].m_lineLength < vVShapes[i].m_dSupportRegionWidth * 10.0 || (fabs(vVShapes[i].m_dAlpha) > 1E-08 && vVShapes[i].m_dAlpha < 0.9))
// 			{
// 				vVShapes[i] = vVShapes[vVShapes.size() - 1];
// 				vVShapes.pop_back();
// 				i--;
// 			}
// 		}
// 		unsigned nIterated = 0; //�����Ĵ���
// 		std::vector<VShapeCluster> vClusters; //�洢��̬�����������
// 		std::vector<VShapeCluster> vCenters; //�洢�ڸ����ƽ������
// 		while (true)
// 		{
// 			nIterated ++;
// 			for (unsigned i = 0; i < vVShapes.size(); i ++) //������������Ķ�̬����
// 			{
// 				if (vVShapes[i].m_lineDirection.norm2() < 1E-08) continue; 
// 				bool bClustered = false; 
// 				for (unsigned j = 0; j < vClusters.size(); j ++)
// 				{
// 					CoordType dCorssAngleCosin = fabs(vVShapes[i].m_lineDirection.dot(vClusters[j].Cluster));
// 					if (dCorssAngleCosin > dCosinCrossAngleThreshold)
// 					{
// 						bClustered = true; 
// 						vCenters[j].Cluster += vVShapes[i].m_lineDirection;
// 						vCenters[j].nQuantity ++; 
// 						vCenters[j].dTotalLength += vVShapes[i].m_lineLength; 
// 						break; 
// 					}
// 				}
// 				if (!bClustered && nIterated == 1) 
// 				{
// 					vClusters.push_back(VShapeCluster(vVShapes[i].m_lineDirection, vVShapes[i].m_lineLength));
// 					vCenters.push_back(VShapeCluster(vVShapes[i].m_lineDirection, vVShapes[i].m_lineLength));
// 				}
// 			}
// 			CoordType dNM = 2.0, dDM = 0.0; 
// 			for (unsigned i = 0; i < vCenters.size(); i ++)
// 			{
// 				//if (vCenters[i].nQuantity < nArrayThreshold) //����������ֵ��ɾ����
// 				//{
// 				//	vClusters[i] = vClusters[vClusters.size() - 1]; vClusters.pop_back();
// 				//	vCenters[i] = vCenters[vCenters.size() - 1]; vCenters.pop_back();
// 				//	i --; 
// 				//	continue; 
// 				//}
// 				vCenters[i].dTotalLength /= vCenters[i].nQuantity;
// 				vCenters[i].Cluster /= vCenters[i].nQuantity;
// 				vCenters[i].Cluster.normalize();
// 				bool bSame = false; 
// 				for (unsigned j = 0; j < i; j ++)
// 				{
// 					CoordType dTempCrossAngleCosin = fabs(vCenters[i].Cluster.dot(vClusters[j].Cluster)); 
// 					if (dTempCrossAngleCosin > dCosinCrossAngleThreshold)
// 					{
// 						bSame = true; 
// 						break; 
// 					}
// 				}
// 				if (!bSame)
// 				{
// 					dNM = min(dNM, fabs(vCenters[i].Cluster.dot(vClusters[i].Cluster)));
// 					vClusters[i] = vCenters[i]; 
// 					vCenters[i].Cluster.x = 0.0;
// 					vCenters[i].Cluster.y = 0.0;
// 					vCenters[i].Cluster.z = 0.0;
// 					vCenters[i].nQuantity = 0;
// 					vCenters[i].dTotalLength = 0.0;
// 				}
// 				else
// 				{
// 					vClusters[i] = vClusters[vClusters.size() - 1]; vClusters.pop_back();
// 					vCenters[i] = vCenters[vCenters.size() - 1]; vCenters.pop_back();
// 					i --; 
// 				}
// 			}//for (unsigned i = 0; i < vClusters.size(); i ++)
// 			if (dNM > dCosinCrossAngleThreshold) break; //
// 		}//while(true)
// 		vCenters.clear(); vCenters.swap(std::vector<VShapeCluster>());
// 		//std::sort(vClusters.begin(), vClusters.end(), VShapeCluster::LengthCompare);
// 		srand((unsigned)time(NULL)); 
// 		for (unsigned i = 0; i < vClusters.size(); i ++) //����������ֵ��ɾ����
// 		{
// 			if (vClusters[i].nQuantity < nArrayThreshold) 
// 			{
// 				vClusters[i] = vClusters[vClusters.size() - 1]; 
// 				vClusters.pop_back();
// 				i --; continue; 
// 			}
// 			BYTE RGB[3] = {(BYTE)(rand() % 255), (BYTE)(rand() % 255), (BYTE)(rand() % 255)};
// 			vClusters[i].setColor(RGB);
// 		}
// 		for (unsigned i = 0; i < vVShapes.size(); i ++) //��������
// 		{
// 			if (vVShapes[i].m_lineLength < vVShapes[i].m_dSupportRegionWidth * 10.0 || (fabs(vVShapes[i].m_dAlpha) > 1E-08 && vVShapes[i].m_dAlpha < 0.9)) goto next;
// 			bool bClustered = false; 
// 			for (unsigned j = 0; j < vClusters.size(); j ++)
// 			{
// 				CoordType dCorssAngleCosin = fabs(vVShapes[i].m_lineDirection.dot(vClusters[j].Cluster));
// 				if (dCorssAngleCosin > dCosinCrossAngleThreshold)
// 				{
// 					bClustered = true; 
// 					memcpy(vVShapes[i].m_leftPlane.m_RGB, vClusters[j].m_RGB, sizeof(BYTE) * 3);
// 					break; 
// 				}
// 			}
// 			if (!bClustered)
// 			{next:
// 				vVShapes[i] = vVShapes[vVShapes.size() - 1];
// 				vVShapes.pop_back();
// 				i--;
// 			}
// 		}
// 		vClusters.clear(); vClusters.swap(std::vector<VShapeCluster>());
// 	}
// 	void Vshapes_GetDigitalLineGraph()//�����߻�ͼ
// 	{
// 		for (unsigned i = 0; i < m_vVShapes.size(); i ++) //����ɸѡˮƽ�߶�
// 		{
// 			CCVector3 ccLeftNorm(m_vVShapes[i].m_leftPlane.m_dNormal), ccRightNorm(m_vVShapes[i].m_rightPlane.m_dNormal), ccZAxis(0.0, 0.0, 1.0);
// 			bool bDelete = false;
// 			if (fabs(ccLeftNorm.dot(ccRightNorm)) > cos(PI * 60.0 / 180.0)) bDelete = true;
// 			if ((fabs(ccLeftNorm.dot(ccZAxis)) < cos(PI * 30.0 / 180.0)) && (fabs(ccRightNorm.dot(ccZAxis)) < cos(PI * 30.0 / 180.0))) bDelete = true;
// 			if (m_vVShapes[i].m_lineLength < m_vVShapes[i].m_dSupportRegionWidth * 3.0) bDelete = true;
// 			if (bDelete)
// 			{
// 				m_vVShapes[i] = m_vVShapes[m_vVShapes.size() - 1];
// 				m_vVShapes.pop_back();
// 				i--;
// 			}
// 		}
// 		for (unsigned i = 0; i < m_vVShapes.size(); i ++) m_vVShapes[i].m_bSaved = false;//�������ñ����ʶ
// 
// 		//��ȡ�߶ηֲ���Χ
// 		CoordType dMaxZ = max(m_vVShapes[0].m_startPoint.z, m_vVShapes[0].m_endPoint.z), dMinZ = min(m_vVShapes[0].m_startPoint.z, m_vVShapes[0].m_endPoint.z);
// 		for (unsigned i = 0; i < m_vVShapes.size(); i ++)
// 		{
// 			dMaxZ = max(dMaxZ, max(m_vVShapes[i].m_startPoint.z, m_vVShapes[i].m_endPoint.z));
// 			dMinZ = min(dMinZ, min(m_vVShapes[i].m_startPoint.z, m_vVShapes[i].m_endPoint.z));
// 		}
// 
// 		for (unsigned i = 0; i < m_vVShapes.size(); i ++) //�����ж�ͶӰ�ص���
// 		{
// 			if (m_vVShapes[i].m_bSaved) continue;
// 			for (unsigned j = 0; j < m_vVShapes.size(); j ++)
// 			{
// 				if (i == j) continue;
// 				CoordType dDistThreshold = (m_vVShapes[i].m_dSupportRegionWidth + m_vVShapes[j].m_dSupportRegionWidth) * 2.0;
// 				//�ж������߶ε�ͶӰ�Ƿ��غ�
// 				CCVector3 ccLineDirection1 = m_vVShapes[i].m_lineDirection; ccLineDirection1.z = 0.0;//�߶�AB�ķ���ʸ��
// 				CCVector3 ccLineDirection2 = m_vVShapes[j].m_lineDirection; ccLineDirection2.z = 0.0;//�߶�CD�ķ���ʸ��
// 				if (fabs(ccLineDirection1.dot(ccLineDirection2)) < cos (PI * 10.0 / 180.0)) continue;
// 				CCVector3 ccTemp0 = m_vVShapes[j].m_endPoint - m_vVShapes[i].m_startPoint, ccTemp1 = m_vVShapes[i].m_endPoint - m_vVShapes[j].m_endPoint;
// 				CCVector3 ccTemp2 = m_vVShapes[j].m_startPoint - m_vVShapes[i].m_startPoint, ccTemp3 = m_vVShapes[i].m_endPoint - m_vVShapes[j].m_startPoint;
// 				ccTemp0.z = 0.0; ccTemp1.z = 0.0; ccTemp2.z = 0.0; ccTemp3.z = 0.0; 
// 				if (ccTemp0.dot(ccTemp1) < 0.0 && ccTemp2.dot(ccTemp3) < 0.0) continue;
// 				CCVector3 ccTempVector = m_vVShapes[j].m_startPoint - m_vVShapes[i].m_startPoint; ccTempVector.z = 0.0;//��ʱ�洢����
// 				CCVector3 ccLineDirection = m_vVShapes[i].m_lineDirection; ccLineDirection.z = 0.0;//�߶�AB�ķ���ʸ��
// 				PCT dC_AB = ccTempVector.norm() * sin(acos(fabs(ccTempVector.dot(ccLineDirection)/ccTempVector.norm()))); //C��AB�ľ���
// 				if (dC_AB > dDistThreshold) continue;
// 				ccTempVector = m_vVShapes[j].m_endPoint - m_vVShapes[i].m_startPoint; ccTempVector.z = 0.0;//��ʱ�洢����
// 				PCT dD_AB =  ccTempVector.norm() * sin(acos(fabs(ccTempVector.dot(ccLineDirection)/ccTempVector.norm()))); //D��AB�ľ���
// 				if (dD_AB > dDistThreshold) continue;
// 
// 				//�߶�ͶӰǰ�߲�
// 				if (fabs((m_vVShapes[j].m_startPoint.z + m_vVShapes[j].m_endPoint.z) / 2.0 - (m_vVShapes[i].m_startPoint.z + m_vVShapes[i].m_endPoint.z) / 2.0) < (dMaxZ - dMinZ) / 10.0) continue;
// 
// 				//�ж������߶��Ƿ��غ�
// 				//if (fabs(m_vVShapes[j].m_endPoint.z - m_vVShapes[i].m_endPoint.z) < dDistThreshold ||
// 				//	fabs(m_vVShapes[j].m_startPoint.z - m_vVShapes[i].m_endPoint.z) < dDistThreshold ||
// 				//	fabs(m_vVShapes[j].m_endPoint.z - m_vVShapes[i].m_startPoint.z) < dDistThreshold ||
// 				//	fabs(m_vVShapes[j].m_startPoint.z - m_vVShapes[i].m_startPoint.z) < dDistThreshold) continue;
// 				//ccTempVector = m_vVShapes[j].m_startPoint - m_vVShapes[i].m_startPoint;//��ʱ�洢����
// 				//ccLineDirection = m_vVShapes[i].m_lineDirection;//�߶�AB�ķ���ʸ��
// 				//dC_AB = ccTempVector.norm() * sin(acos(fabs((ccTempVector).dot(ccLineDirection)/ccTempVector.norm()))); //C��AB�ľ���
// 				//if (dC_AB < dDistThreshold) continue;
// 				//ccTempVector = m_vVShapes[j].m_endPoint - m_vVShapes[i].m_startPoint;//��ʱ�洢����
// 				//dD_AB =  ccTempVector.norm() * sin(acos(fabs((ccTempVector).dot(ccLineDirection)/ccTempVector.norm()))); //D��AB�ľ���
// 				//if (dD_AB < dDistThreshold) continue;
// 
// 				m_vVShapes[i].m_bSaved = true; m_vVShapes[j].m_bSaved = true;
// 				break;
// 			}
// 		}
// 		for (unsigned i = 0; i < m_vVShapes.size(); i ++) //����ɾ�����ص��߶�
// 		{
// 			if (!m_vVShapes[i].m_bSaved)
// 			{
// 				m_vVShapes[i] = m_vVShapes[m_vVShapes.size() - 1];
// 				m_vVShapes.pop_back();
// 				i--;
// 			}
// 		}
// 	}
// 	void AdjacentPolyLineProcessing(template_Struct_Poly_Shape<CoordType, PointType> &polyShape, PCT dSupportPlaneNormal[], unsigned ID0, unsigned ID1, unsigned ID2, unsigned ID3)
// 	{
// 		//�ཻ�ߵĴ���
// 		CCVector3 ccPt0 = polyShape.m_Vertex[ID0]; CCVector3 ccPt1 = polyShape.m_Vertex[ID1];
// 		CCVector3 ccPt2 = polyShape.m_Vertex[ID2]; CCVector3 ccPt3 = polyShape.m_Vertex[ID3]; 
// 		/*����AB�ϵ��߶ν��㣨����ͶӰ���㣩*/
// 		PCT A[9], B[9], D[3]; //�洢һԪ���η��̵�ϵ������ͳ�������-����ƽ���ƽ�淽��
// 		CCVector3 ccTempVector, ccTempDirection; //��ʱ�洢��ƽ��ķ�ʸ
// 		CCVector3 ccTempCross(dSupportPlaneNormal); //���߶�AB��CD�����
// 		/*CD�������ƽ��*/
// 		ccTempDirection = ccPt3 - ccPt2; ccTempDirection.normalize(); ccTempVector = ccTempCross.cross(ccTempDirection); ccTempVector.normalize();
// 		B[0] = A[0] = ccTempVector.x; B[1] = A[1] = ccTempVector.y; B[2] = A[2] = ccTempVector.z;
// 		D[0] = A[0] * ccPt2.x + A[1] * ccPt2.y + A[2] * ccPt2.z;
// 		/*AB�������ƽ��*/
// 		ccTempDirection = ccPt1 - ccPt0; ccTempDirection.normalize(); ccTempVector = ccTempCross.cross(ccPt1 - ccPt0); ccTempVector.normalize();
// 		B[3] = A[3] = ccTempVector.x; B[4] = A[4] = ccTempVector.y; B[5] = A[5] = ccTempVector.z;
// 		D[1] = A[3] * ccPt0.x + A[4] * ccPt0.y + A[5] * ccPt0.z;
// 		/*��AB��ʸΪ�����ƽ��*/
// 		B[6] = A[6] = ccTempCross.x; B[7] = A[7] = ccTempCross.y; B[8] = A[8] = ccTempCross.z;
// 		D[2] = A[6] * ccPt0.x + A[7] * ccPt0.y + A[8] * ccPt0.z;
// 		CCVector3 ccCrossPoint; //��õ��¶˵�����
// 		matrixInvert_GaussJordan(A, 3); //ϵ������A����
// 		ccCrossPoint = matrixMultiply_Vector_3D<PCT, CCVector3>(A, D); //X=-A`'D
// 		/*ȷ��AB�䶯��ĵ�λ*/
// 		if (max((ccPt0 - ccCrossPoint).norm(), (ccPt1 - ccCrossPoint).norm()) < (ccPt1 - ccPt0).norm() * 1.1 && 
// 			max((ccPt2 - ccCrossPoint).norm(), (ccPt3 - ccCrossPoint).norm()) < (ccPt3 - ccPt2).norm() * 1.1)
// 		{//�ж��ĸ��˵��뽻�����
// 			if ((ccPt0 - ccCrossPoint).norm() < (ccPt1 - ccCrossPoint).norm()) ccPt0 = polyShape.m_Vertex[ID0] = ccCrossPoint; //�����ý�
// 			else ccPt1 = polyShape.m_Vertex[ID1] = ccCrossPoint; //�յ���ý�
// 			if ((ccPt2 - ccCrossPoint).norm() < (ccPt3 - ccCrossPoint).norm()) ccPt2 = polyShape.m_Vertex[ID2] = ccCrossPoint; //�����ý�
// 			else ccPt3 = polyShape.m_Vertex[ID3] = ccCrossPoint; //�յ���ý�
// 		}
// 	}
// 	PointType GetProjectionPoint_Point_LineSegment(PointType pPoint, template_Struct_V_Shape<CoordType, PointType> pLine)
// 	{
// 		PointType projectionPT = pPoint - pLine.m_startPoint;
// 		projectionPT = pLine.m_startPoint + projectionPT.dot(pLine.m_lineDirection) * pLine.m_lineDirection;
// 		return projectionPT;
// 	}
// 	PointType getProjectionPoint_Point_Plane(CoordType dPlane[], PointType ccPoint)
// 	{
// 		CoordType dProjectionDist = dPlane[0] * ccPoint.x + dPlane[1] * ccPoint.y + dPlane[2] * ccPoint.z + dPlane[3]; //ͶӰ���� 
// 		return PointType(ccPoint.x - dProjectionDist * dPlane[0], ccPoint.y - dProjectionDist * dPlane[1], ccPoint.z - dProjectionDist * dPlane[2]);
// 	}
// 	template<typename CloudType>
// 	void GetModel_from_VShapes(std::vector<template_Struct_V_Shape<CoordType, PointType>> &vs_Output, std::vector<template_Struct_Poly_Shape<CoordType, PointType>> &vec_poly_model, CloudType LinkCloud)//��ȡ�컨���ǽ����
// 	{
// 		unsigned uCeilingID = this->m_vPlanes.size() + 1; CoordType dMaxArea = 0.0;
// 		for (unsigned i = 0; i < this->m_vPlanes.size(); i ++)//Ѱ���������ƽ��
// 		{
// 			CoordType dTempArea = this->m_vPlanes[i].m_vPlanePointIDs.size() * this->m_vPlanes[i].m_dProjectionPixelWidth;
// 			if (dMaxArea < dTempArea) {dMaxArea = dTempArea; uCeilingID = this->m_vPlanes[i].m_uPlaneID;}
// 		}
// 		if (uCeilingID > this->m_vPlanes.size()) return;
// 
// 		LinkCloud->setUnselected(0, LinkCloud->size());
// 		for (unsigned i = 0; i < this->m_vPlanes[uCeilingID - 1].m_vPlanePointIDs.size(); i ++) LinkCloud->setSelected(this->m_vPlanes[uCeilingID - 1].m_vPlanePointIDs[i], true);//����컨��ƽ��
// 
// 		union//�컨����ѧ����
// 		{
// 			struct
// 			{
// 				PointType ccCeilingNormal; //��ʸ
// 				PCT dCeilingD; //ƽ�淽�̳�����
// 			};
// 			CoordType dCeilingPlane[4]; //ƽ�淽��
// 		};
// 		memcpy(dCeilingPlane, this->m_vPlanes[uCeilingID - 1].m_dNormal, sizeof(CoordType) * 4);
// 		std::vector<template_Struct_V_Shape<CoordType, PointType>> vs_column;
// 		for (unsigned i = 0; i < this->m_vVShapes.size(); i ++) //��ȡǽ�����������컨��ӽ���ֱ���ཻ���߶�
// 		{
// 			if (fabs(this->m_vVShapes[i].m_leftPlane.m_normal.dot(this->m_vVShapes[i].m_rightPlane.m_normal)) > cos(PI * 67.5 / 180.0)) continue; 
// 			if (fabs(this->m_vVShapes[i].m_lineDirection.dot(ccCeilingNormal)) > cos(PI * 7.5 / 180.0))
// 			{
// 				CoordType dDistThreshold = this->m_vVShapes[i].m_dSupportRegionWidth * 2.0;
// 				if ((fabs(this->m_vVShapes[i].m_startPoint.dot(ccCeilingNormal) + dCeilingD) < dDistThreshold && (this->m_vVShapes[i].m_endPoint.dot(ccCeilingNormal) + dCeilingD) < 0.0) ||
// 					(fabs(this->m_vVShapes[i].m_endPoint.dot(ccCeilingNormal) + dCeilingD) < dDistThreshold && (this->m_vVShapes[i].m_startPoint.dot(ccCeilingNormal) + dCeilingD) < 0.0))
// 				{
// 					PointType ccCrossPT;
// 					this->m_vVShapes[i].getPlaneCrossPoint(ccCrossPT, dCeilingPlane);
// 					if ((this->m_vVShapes[i].m_startPoint - ccCrossPT).norm() < (this->m_vVShapes[i].m_endPoint - ccCrossPT).norm()) this->m_vVShapes[i].m_startPoint = ccCrossPT;
// 					else this->m_vVShapes[i].m_endPoint = ccCrossPT;
// 					this->m_vVShapes[i].getVShapeSupportRegionVertex();
// 					vs_column.push_back(this->m_vVShapes[i]);
// 				}
// 			}
// 		}
// 		std::vector<bool> bColumnIsTrue(vs_column.size(), false);//��ʾÿ��ǽ�����Ƿ����ཻ���컨��߽���
// 		std::vector<template_Struct_V_Shape<CoordType, PointType>> vs_celing;
// 		for (unsigned i = 0; i < this->m_vVShapes.size(); i ++) //��ȡ�컨��ı߽��߶�
// 		{
// 			if (fabs(this->m_vVShapes[i].m_leftPlane.m_normal.dot(this->m_vVShapes[i].m_rightPlane.m_normal)) > cos(PI * 67.5 / 180.0)) continue; 
// 			if (this->m_vVShapes[i].m_leftPlane.m_uPlaneID == uCeilingID || this->m_vVShapes[i].m_rightPlane.m_uPlaneID == uCeilingID)
// 			{
// 				vector<PointType> vCrossPT;
// 				for (unsigned j = 0; j < vs_column.size(); j ++)
// 				{
// 					if (vs_column[j].m_leftPlane.m_uPlaneID != this->m_vVShapes[i].m_leftPlane.m_uPlaneID &&
// 						vs_column[j].m_leftPlane.m_uPlaneID != this->m_vVShapes[i].m_rightPlane.m_uPlaneID &&
// 						vs_column[j].m_rightPlane.m_uPlaneID != this->m_vVShapes[i].m_leftPlane.m_uPlaneID &&
// 						vs_column[j].m_rightPlane.m_uPlaneID != this->m_vVShapes[i].m_rightPlane.m_uPlaneID) continue;//������ʱ����
// 					if (fabs(vs_column[j].m_startPoint.dot(ccCeilingNormal) + dCeilingD) < fabs(vs_column[j].m_endPoint.dot(ccCeilingNormal) + dCeilingD))
// 					{
// 						PointType ccTempVector = vs_column[j].m_startPoint - this->m_vVShapes[i].m_startPoint;
// 						CoordType dTempStartPointProjectionDist = ccTempVector.norm();
// 						if (dTempStartPointProjectionDist < (this->m_vVShapes[i].m_dSupportRegionWidth / 5.0)) {vCrossPT.push_back(vs_column[j].m_startPoint); bColumnIsTrue[j] = true; continue;}
// 						ccTempVector.normalize(); 
// 						CoordType dTempAngle = fabs(ccTempVector.dot(this->m_vVShapes[i].m_lineDirection));
// 						if (dTempAngle > cos(PI * 89.0 / 180.0)) {vCrossPT.push_back(vs_column[j].m_startPoint); bColumnIsTrue[j] = true; continue;}
// 						dTempStartPointProjectionDist *= sin(acos(dTempAngle));
// 						if (dTempStartPointProjectionDist < (this->m_vVShapes[i].m_dSupportRegionWidth / 5.0)) {vCrossPT.push_back(vs_column[j].m_startPoint); bColumnIsTrue[j] = true; continue;}
// 					}
// 					else
// 					{
// 						PointType ccTempVector = vs_column[j].m_endPoint - this->m_vVShapes[i].m_startPoint;
// 						CoordType dTempEndPointProjectionDist = ccTempVector.norm(); 
// 						if (dTempEndPointProjectionDist < (this->m_vVShapes[i].m_dSupportRegionWidth / 5.0)) {vCrossPT.push_back(vs_column[j].m_endPoint); bColumnIsTrue[j] = true; continue;}
// 						ccTempVector.normalize(); 
// 						CoordType dTempAngle = fabs(ccTempVector.dot(this->m_vVShapes[i].m_lineDirection));
// 						if (dTempAngle > cos(PI * 89.0 / 180.0)) {vCrossPT.push_back(vs_column[j].m_endPoint); bColumnIsTrue[j] = true; continue;}
// 						dTempEndPointProjectionDist *= sin(acos(dTempAngle));
// 						if (dTempEndPointProjectionDist < (this->m_vVShapes[i].m_dSupportRegionWidth / 5.0)) {vCrossPT.push_back(vs_column[j].m_endPoint); bColumnIsTrue[j] = true; continue;}
// 					}
// 				}
// 				if (vCrossPT.size()) 
// 				{
// 					for (unsigned j = 0; j < vCrossPT.size(); j ++)
// 					{
// 						if (max((this->m_vVShapes[i].m_startPoint - vCrossPT[j]).norm(), (this->m_vVShapes[i].m_endPoint - vCrossPT[j]).norm()) > this->m_vVShapes[i].m_lineLength)
// 						{
// 							if ((this->m_vVShapes[i].m_startPoint - vCrossPT[j]).norm() < (this->m_vVShapes[i].m_endPoint - vCrossPT[j]).norm()) this->m_vVShapes[i].m_startPoint = vCrossPT[j];
// 							else this->m_vVShapes[i].m_endPoint = vCrossPT[j];
// 						}
// 					}
// 					this->m_vVShapes[i].getVShapeSupportRegionVertex();
// 					vs_celing.push_back(this->m_vVShapes[i]);
// 				}
// 				vCrossPT.clear(); vCrossPT.swap(vector<PointType> ());
// 			}
// 		}
// 		for (unsigned j = 0; j < vs_celing.size(); j ++) vs_Output.push_back(vs_celing[j]); //������ȡ���컨��߽���
// 		for (unsigned j = 0; j < vs_column.size(); j ++) //ɾ�����컨��߽��߲��ཻ��ǽ����
// 		{
// 			if (!bColumnIsTrue[j]) 
// 			{
// 				vs_column[j] = vs_column[vs_column.size() - 1]; 
// 				bColumnIsTrue[j] = bColumnIsTrue[bColumnIsTrue.size() - 1];
// 				vs_column.pop_back(); bColumnIsTrue.pop_back();
// 				j --;
// 			}
// 		}
// 		bColumnIsTrue.clear(); bColumnIsTrue.swap(std::vector<bool> ());
// 		for (unsigned j = 0; j < vs_column.size(); j ++) vs_Output.push_back(vs_column[j]); //�������컨��߽����ཻ��ǽ����
// 		vs_column.clear(); vs_column.swap(std::vector<template_Struct_V_Shape<CoordType, PointType>> ());
// 
// 		////////////////////////////////////////////////////////////////////////////
// 		/*������ȡ�ݶ������*/
// 		template_Struct_Poly_Shape<CoordType, PointType> poly_ceiling;//�ݶ������
// 		poly_ceiling.m_SupportPlane = this->m_vPlanes[uCeilingID - 1];
// 		poly_ceiling.m_seperator.push_back(poly_ceiling.size()); 
// 		// ��ȡ�����߶Σ��������
// 		PCT dTempLength = vs_celing[0].m_lineLength; unsigned seed_ID = 0, origin_seed_ID = 0;
// 		for (unsigned j = 1; j < vs_celing.size(); j ++) if (dTempLength < vs_celing[j].m_lineLength) {seed_ID = j; dTempLength = vs_celing[j].m_lineLength;}
// 		poly_ceiling.addLine(vs_celing[seed_ID]);
// 		vs_celing[seed_ID].m_bSaved = true;
// 		// �������߶��յ㿪ʼ����
// 		origin_seed_ID = seed_ID;
// 		unsigned latest_Growed_ID = seed_ID;
// 		while (true)
// 		{
// 			PointType ccSeed = poly_ceiling.m_Vertex[poly_ceiling.size() - 1];//���ӵ�
// 			PointType ccCurrentLine = ccSeed - poly_ceiling.m_Vertex[poly_ceiling.size() - 2]; ccCurrentLine.normalize();//�����߶�
// 			dTempLength = vs_celing[origin_seed_ID].m_lineLength * 1E10; unsigned growed_ID = vs_celing.size() + 1;
// 			for (unsigned j = 0; j < vs_celing.size(); j ++)
// 			{
// 				if (j == seed_ID || j == latest_Growed_ID || vs_celing[j].m_bSaved) continue;
// 				PCT dTemp = min((ccSeed - vs_celing[j].m_startPoint).norm(), (ccSeed - vs_celing[j].m_endPoint).norm());
// 				if (dTemp > dTempLength) continue;
// 				PCT dCrossAngle = fabs(ccCurrentLine.dot(vs_celing[j].m_lineDirection));
// 				if (dCrossAngle < cos(PI * 87.0 / 180.0) || dCrossAngle > cos(PI * 3.0 / 180.0)) { dTempLength = dTemp; growed_ID = j; }
// 				else continue;
// 			}
// 			if (growed_ID > vs_celing.size()) break;					
// 			latest_Growed_ID = seed_ID; seed_ID = growed_ID; vs_celing[growed_ID].m_bSaved = true;
// 			PCT dTenpDistStart = (ccSeed - vs_celing[growed_ID].m_startPoint).norm();
// 			PCT dTenpDistEnd = (ccSeed - vs_celing[growed_ID].m_endPoint).norm();
// 			if (dTenpDistStart > dTenpDistEnd) {poly_ceiling.addVertex(vs_celing[growed_ID].m_endPoint); poly_ceiling.addVertex(vs_celing[growed_ID].m_startPoint);}
// 			else {poly_ceiling.addVertex(vs_celing[growed_ID].m_startPoint); poly_ceiling.addVertex(vs_celing[growed_ID].m_endPoint);}
// 		}
// 		//�����ཻ�ߵĴ���
// 		for (unsigned j = 0; j < poly_ceiling.size() / 2 - 1; j ++) AdjacentPolyLineProcessing(poly_ceiling, dCeilingPlane, j * 2, j * 2 + 1, (j + 1) * 2 + 0, (j + 1) * 2 + 1);
// 		AdjacentPolyLineProcessing(poly_ceiling, dCeilingPlane, 0, 1, poly_ceiling.size() - 2, poly_ceiling.size() - 1);
// 		//��ֱ�ǱߵĹ���
// 		for (unsigned j = 1; j < poly_ceiling.size() - 1; j ++) 
// 		{
// 			template_Struct_V_Shape<CoordType, PointType> ccLine1, ccLine2;
// 			ccLine1.m_startPoint = poly_ceiling.m_Vertex[j - 1]; ccLine1.m_endPoint = poly_ceiling.m_Vertex[j]; ccLine1.update();
// 			ccLine2.m_startPoint = poly_ceiling.m_Vertex[j]; ccLine2.m_endPoint = poly_ceiling.m_Vertex[j + 1]; ccLine2.update();
// 			if (fabs(ccLine1.m_lineDirection.dot(ccLine2.m_lineDirection)) > cos(PI * 87.0 / 180.0) && fabs(ccLine1.m_lineDirection.dot(ccLine2.m_lineDirection)) < cos(PI * 3.0 / 180.0))
// 			{
// 				template_Struct_V_Shape<CoordType, PointType> ccLine3; unsigned uPos = 0;
// 				if (j + 2 < poly_ceiling.size()) {ccLine3.m_startPoint = poly_ceiling.m_Vertex[j + 1]; ccLine3.m_endPoint = poly_ceiling.m_Vertex[j + 2]; ccLine3.update(); uPos = j + 1;}
// 				else {ccLine3.m_startPoint = poly_ceiling.m_Vertex[0]; ccLine3.m_endPoint = poly_ceiling.m_Vertex[1]; ccLine3.update(); uPos = 0;}
// 				if (ccLine1.m_lineLength < ccLine3.m_lineLength) poly_ceiling.m_Vertex[uPos] = GetProjectionPoint_Point_LineSegment(ccLine1.m_endPoint, ccLine3);
// 				else poly_ceiling.m_Vertex[j] = GetProjectionPoint_Point_LineSegment(ccLine3.m_startPoint, ccLine1);
// 			}
// 		}
// 		//ͬһ���߶ζ������εĴ���
// 		for (unsigned j = 1; j < poly_ceiling.size() - 1; j ++) 
// 		{
// 			template_Struct_V_Shape<CoordType, PointType> ccLine1, ccLine2;
// 			ccLine1.m_startPoint = poly_ceiling.m_Vertex[j - 1]; ccLine1.m_endPoint = poly_ceiling.m_Vertex[j]; ccLine1.update();
// 			ccLine2.m_startPoint = poly_ceiling.m_Vertex[j]; ccLine2.m_endPoint = poly_ceiling.m_Vertex[j + 1]; ccLine2.update();
// 			if (fabs(ccLine1.m_lineDirection.dot(ccLine2.m_lineDirection)) > cos(PI * 1.0 / 180.0)) {poly_ceiling.m_Vertex.erase(poly_ceiling.m_Vertex.begin() + j); j --; continue;}
// 		}
// 		//�ص��˵�ĺϲ�
// 		for (unsigned j = 1; j < poly_ceiling.size(); j ++) if ((poly_ceiling.m_Vertex[j] - poly_ceiling.m_Vertex[j - 1]).norm() < 1E-03) {poly_ceiling.m_Vertex.erase(poly_ceiling.m_Vertex.begin() + j); j --; continue;}
// 		poly_ceiling.m_seperator.push_back(poly_ceiling.size()); 
// 		////////////////////////////////////////////////////////////////////////
// 		/*�ذ���ȡ*/
// 		unsigned uFloorID = this->m_vPlanes.size() + 1; dMaxArea = 0.0;
// 		for (unsigned i = 0; i < this->m_vPlanes.size(); i ++)//ƽ�����ݶ����������ƽ��
// 		{
// 			CoordType dTempArea = this->m_vPlanes[i].m_vPlanePointIDs.size() * this->m_vPlanes[i].m_dProjectionPixelWidth;
// 			if (this->m_vPlanes[i].m_uPlaneID != uCeilingID && fabs(this->m_vPlanes[i].m_normal.dot(ccCeilingNormal)) > cos(PI * 3.0 / 180.0) && dMaxArea < dTempArea) 
// 			{dMaxArea = dTempArea; uFloorID = this->m_vPlanes[i].m_uPlaneID;}
// 		}
// 		if (uFloorID > this->m_vPlanes.size()) return; //û��ʶ�𵽵ذ壬��Ҳû��ǽ��
// 		template_Struct_Poly_Shape<CoordType, PointType> poly_floor;//�ذ�����
// 		poly_floor.m_SupportPlane = this->m_vPlanes[uFloorID - 1];
// 		poly_floor.m_seperator.push_back(poly_floor.size()); 
// 		for (unsigned i = 0; i < poly_ceiling.size(); i ++) poly_floor.addVertex(getProjectionPoint_Point_Plane(this->m_vPlanes[uFloorID - 1].m_dNormal, poly_ceiling.m_Vertex[i]));
// 		poly_floor.m_seperator.push_back(poly_floor.size()); 
// 		for (unsigned i = 0; i < this->m_vPlanes[uFloorID - 1].m_vPlanePointIDs.size(); i ++) LinkCloud->setSelected(this->m_vPlanes[uFloorID - 1].m_vPlanePointIDs[i], true);//��ǵذ�ƽ��
// 		////////////////////////////////////////////////////////////////////////////
// 		/*ǽ��*/
// 		for (unsigned i = 0; i < poly_ceiling.size() - 1; i ++)
// 		{
// 			unsigned uWallID = this->m_vPlanes.size() + 1;
// 			for (unsigned j = 0; j < this->m_vPlanes.size(); j ++)//for (unsigned j = 0; j < vs_celing.size(); j ++)
// 			{
// 				if (fabs(this->m_vPlanes[j].m_normal.dot(ccCeilingNormal)) > cos(PI * 87.0 / 180.0)) continue;
// 				if (fabs(poly_ceiling.m_Vertex[i].dot(this->m_vPlanes[j].m_normal) + this->m_vPlanes[j].m_dD) < this->m_vPlanes[j].m_dRMS * 10.0 &&
// 					fabs(poly_ceiling.m_Vertex[i + 1].dot(this->m_vPlanes[j].m_normal) + this->m_vPlanes[j].m_dD) < this->m_vPlanes[j].m_dRMS * 10.0 &&
// 					fabs(poly_floor.m_Vertex[i].dot(this->m_vPlanes[j].m_normal) + this->m_vPlanes[j].m_dD) < this->m_vPlanes[j].m_dRMS * 10.0 &&
// 					fabs(poly_floor.m_Vertex[i + 1].dot(this->m_vPlanes[j].m_normal) + this->m_vPlanes[j].m_dD) < this->m_vPlanes[j].m_dRMS * 10.0)
// 				{
// 					uWallID = this->m_vPlanes[j].m_uPlaneID;
// 					break;
// 				}
// 				//if (fabs((poly_ceiling.m_Vertex[i + 1] - poly_ceiling.m_Vertex[i]).dot(vs_celing[j].m_lineDirection)) > cos(PI * 1.0 / 180.0))
// 				//{
// 				//	if ((poly_ceiling.m_Vertex[i] - vs_celing[j].m_startPoint).norm() < 1E-02 || 
// 				//		(poly_ceiling.m_Vertex[i + 1] - vs_celing[j].m_startPoint).norm() < 1E-02 || 
// 				//		(poly_ceiling.m_Vertex[i] - vs_celing[j].m_endPoint).norm() < 1E-02 || 
// 				//		(poly_ceiling.m_Vertex[i + 1] - vs_celing[j].m_endPoint).norm() < 1E-02)
// 				//	{
// 				//		uWallID = (vs_celing[j].m_leftPlane.m_uPlaneID != uCeilingID) ? vs_celing[j].m_leftPlane.m_uPlaneID : vs_celing[j].m_rightPlane.m_uPlaneID;
// 				//		break;
// 				//	}
// 				//}
// 			}
// 			template_Struct_Poly_Shape<CoordType, PointType> poly_wall;//ǽ������
// 			if (uWallID <= this->m_vPlanes.size()) 
// 			{
// 				poly_wall.m_SupportPlane = this->m_vPlanes[uWallID - 1];
// 				for (unsigned j = 0; j < this->m_vPlanes[uWallID - 1].m_vPlanePointIDs.size(); j ++) LinkCloud->setSelected(this->m_vPlanes[uWallID - 1].m_vPlanePointIDs[j], true);//���ǽ��ƽ��
// 			}
// 			else
// 			{
// 				poly_wall.m_SupportPlane.m_RGB[0] = (BYTE)(155 + rand() % 100);
// 				poly_wall.m_SupportPlane.m_RGB[1] = (BYTE)(155 + rand() % 100);
// 				poly_wall.m_SupportPlane.m_RGB[2] = (BYTE)(155 + rand() % 100);
// 			}
// 			poly_wall.m_seperator.push_back(poly_wall.size()); 
// 			poly_wall.addVertex(poly_ceiling.m_Vertex[i]);
// 			poly_wall.addVertex(poly_ceiling.m_Vertex[i + 1]);
// 			poly_wall.addVertex(poly_floor.m_Vertex[i + 1]);
// 			poly_wall.addVertex(poly_floor.m_Vertex[i]);
// 			poly_wall.m_seperator.push_back(poly_wall.size()); 
// 			vec_poly_model.push_back(poly_wall); poly_wall.clear();
// 		}
// 		//////////////////////////////////////////////////////////////////////////
// 		/*���ؽ��*/
// 		vs_celing.clear(); vs_celing.swap(std::vector<template_Struct_V_Shape<CoordType, PointType>>());
// 		vs_column.clear(); vs_column.swap(std::vector<template_Struct_V_Shape<CoordType, PointType>>());
// 		vec_poly_model.push_back(poly_ceiling);
// 		vec_poly_model.push_back(poly_floor); 
// 		poly_ceiling.clear();poly_floor.clear();
// 	}
// };
		
class PlaneDetection
{
public:
    
    PlaneDetection();
    ~PlaneDetection();
    
    void createVoxelization( pcl::PointCloud<PointT>::Ptr inputPoints, const double& initRes
                             , const double& lowestRes, const double& distThr );

    /**
     * \brief proposed by Minglei Li -> Adaptive Multi-level Octree Based Region Growing
     */
//     void runPlaneExtraction_AMG();
//     void getPlanarFeature_AMG();
    
    
    
    /**
     * \brief proposed by VO -> Octree Based Region Growing
     */
    void runPlaneExtraction_OBR(pcl::PointCloud<PointT>::Ptr inputPoints, const double& angleThreshold /*degree*/, const double& resThreshold /*m*/);
    void getPlanarFeature_OBR();
    double getAverageDistance();
    void saliencyFeatureEstimation(std::vector<VoxelInfo> &cellInfos);
    void voxelBasedRegionGrowing(const std::vector<VoxelInfo> &cellInfos, std::vector<SegmentInfo> &segmentInfos);
			
public:
    
    ///< octree based region growing parameters
    pcl::octree::OctreePointCloudAdjacency<PointT>::Ptr octree_;
    pcl::PointCloud<PointT>::Ptr points_;
    double init_resolution_;
    double lowest_resolution_;
    double dist_threshold_;
    
    double res_threshold_;
    double angle_threshold_;
    
};
#endif //_PLANE_DETECTION_H_