


#include "stasm.hpp"

static const int  NMODELS_MAX = 2;  // max number of stacked models

typedef struct SEARCH_IMAGES    // the images for one pyr level during a search
    {
    Image  Img;                 // the input image, scaled to current pyramid level
    Mat    Grads;               // image gradients
    }
SEARCH_IMAGES;

//-----------------------------------------------------------------------------
static int
nGetProfWidthFromModel (int iPoint, const ASM_LEVEL_DATA &AsmLev)
{
return nGetProfWidth(AsmLev.Profs[iPoint].ncols(), AsmLev.ProfSpecs[iPoint]);
}

//-----------------------------------------------------------------------------
// Return the Mahalanobis distance between the image profile and
// model profile at the given iPoint and offset.
// ix and iy are the offset and orthogonal offset wrt iPoint.

static double
GetProfDist (const SEARCH_IMAGES &SearchImgs,         // in: all args
             const int iPoint, const int ix, const int iy,
             const ASM_LEVEL_DATA &AsmLev, const SHAPE &Shape,
             const double SigmoidScale)
{
// It is quicker to allocate Prof statically and call dim() each time because
// most of the time the dimension is the same as the previous profile
// (and so we avoid a malloc and free every time this routine is called).
static Vec Prof;
Prof.dim(1, AsmLev.Profs[iPoint].ncols());

if (IS_2D(AsmLev.ProfSpecs[iPoint])) // two dimensional profile?
    {
    const int nProfWidth = nGetProfWidthFromModel(iPoint, AsmLev);
    Get2dProf(Prof,
        AsmLev.ProfSpecs[iPoint], SearchImgs.Grads,
        Shape, iPoint, ix, iy, nProfWidth, SigmoidScale);
    }
else
    Get1dProf(Prof,
        AsmLev.ProfSpecs[iPoint], SearchImgs.Img, iPoint, ix);

// The following code is equivalent to
//      return (Prof - ModelMeanProf) * Covar * (Prof - ModelMeanProf).t();
// but is optimized for speed.

Prof -= AsmLev.Profs[iPoint]; // for efficiency, use "-=" rather than "="
const Mat *pCovar = &AsmLev.Covars[iPoint];
if (pCovar->nrows() > 1)    // TODO ?
    return xAx(Prof, *pCovar);
else
    return Sparse_xAx(Prof, AsmLev.SparseCovars[iPoint]);
}

//-----------------------------------------------------------------------------
// Return offsets (from iPoint) ixBest and iyBest at the best profile match.
// Table to explain what x and y directions mean:
//
//             1D                       2D
//    x    offset along whisker     horiz offset
//    y    always 0                 vert offset

static void
FindBestMatchingProf (int &ixBest,      // ou
    int &iyBest,                        // out
    int iPoint,                         // in
    const SHAPE &Shape,                 // in
    const LANDMARK LandTab[],           // in
    const SEARCH_IMAGES &SearchImgs,    // in
    const ASM_LEVEL_DATA &Model,        // in
    int nPixSearch,                     // in
    double SigmoidScale,                // in
    bool fExplicitPrevNext)             // in
{
int nyMaxOffset = 0;
unsigned ProfSpec = Model.ProfSpecs[iPoint];
if (IS_2D(ProfSpec))            // two dimensional profile?
    nyMaxOffset = nPixSearch;   // if so, we need to offset in y dir also
else
    PrepareProf1D(SearchImgs.Img, Shape, ProfSpec, LandTab, iPoint,
        nGetProfWidthFromModel(iPoint, Model) + 2 * nPixSearch,
        0, fExplicitPrevNext);

double BestFit = DBL_MAX;
ixBest = 0, iyBest = 0;

for (int iy = -nyMaxOffset; iy <= nyMaxOffset; iy++)
    for (int ix = -nPixSearch; ix <= nPixSearch; ix++)
        {
        double Fit = GetProfDist(SearchImgs, iPoint, ix, iy,
                                 Model, Shape, SigmoidScale);

        // Test for a new best fit. We test using "<=" instead of just "<"
        // so if there is an exact match then ixBest=0 i.e. no change.

        if ((ix <= 0 && iy <= 0)? Fit <= BestFit:  Fit < BestFit)
            {
            ixBest = ix;
            iyBest = iy;
            BestFit = Fit;
            }
        }
}

//-----------------------------------------------------------------------------
// Update Shape to SuggestedShape by matching the image profile to
// the model profile at each landmark.
// Returns number of landmarks that are "good".

static int
GetSuggestedShape (SHAPE& SuggestedShape,           // io
    const SHAPE& Shape,                             // in
    const SEARCH_IMAGES &SearchImgs,                // in
    const ASM_LEVEL_DATA &Model,                    // in
    const LANDMARK LandTab[],                       // in
    int nPixSearch, int nPixSearch2d,               // in
    double SigmoidScale, bool fExplicitPrevNext)    // in
{
int nGoodLandmarks = 0;
int ixBest, iyBest;
int nPoints = SuggestedShape.nrows();

for (int iPoint = 0; iPoint < nPoints; iPoint++)
    {
    const unsigned ProfSpec = Model.ProfSpecs[iPoint];
    if (IS_2D(ProfSpec))         // two dimensional profile?
        nPixSearch = nPixSearch2d;

    FindBestMatchingProf(ixBest, iyBest,
        iPoint, Shape, LandTab, SearchImgs, Model,
        nPixSearch, SigmoidScale, fExplicitPrevNext);

    // set SuggestedShape(iPoint) to best offset from current position

    if (IS_2D(ProfSpec))         // two dimensional profile?
        {
        // x,y orthogonal to image sides (not to whisker)

        SuggestedShape(iPoint, VX) = Shape(iPoint, VX) - ixBest;
        SuggestedShape(iPoint, VY) = Shape(iPoint, VY) - iyBest;
        if (ABS(ixBest) + ABS(iyBest) <= nPixSearch)
            nGoodLandmarks++;
        }
    else    // one dimensional profile: must move point along the whisker
        {
        double DeltaX = 0, DeltaY = 0;
        if (ixBest || iyBest)
            GetProfStepSize(DeltaX, DeltaY,
                Shape, iPoint, LandTab, fExplicitPrevNext);

        SuggestedShape(iPoint, VX) = GetX(Shape(iPoint, VX),
                                          ixBest, iyBest, DeltaX, DeltaY);
        SuggestedShape(iPoint, VY) = GetY(Shape(iPoint, VY),
                                          ixBest, iyBest, DeltaX, DeltaY);
        if (ABS(ixBest) <= nPixSearch/2)
            nGoodLandmarks++;
        }
    }
return nGoodLandmarks;
}

//-----------------------------------------------------------------------------
// do an ASM search at one level in the image pyramid

static void
AsmLevSearch (SHAPE &Shape,     // io
    SEARCH_IMAGES &SearchImgs,  // in
    const ASM_MODEL &Model,     // in
    int iLev,                   // in
    const LANDMARK LandTab[])   // in
{
int   iter = 0, nGoodLandmarks = 0;
SHAPE SuggestedShape(Shape);    // shape after profile matching

// The shape params, initialized to 0.  The original formulation called for
// this to be set to 0 each time we run the model but we get slightly
// better results if we remember the shape params from the previous run.
// Thus this is outside the loop.

Vec b(Model.EigVecs.nrows());

int nPoints = Shape.nrows();

while ((iter < Model.AsmLevs[iLev].nMaxSearchIters) &&
        (nGoodLandmarks <= (Model.AsmLevs[iLev].nQualifyingDisp * nPoints)/100))
    {
    // estimate the best SuggestedShape by profile matching the landmarks in Shape

    nGoodLandmarks = GetSuggestedShape(SuggestedShape,
                                       Shape, SearchImgs,
                                       Model.AsmLevs[iLev], LandTab,
                                       Model.nPixSearch, Model.nPixSearch2d,
                                       Model.SigmoidScale, Model.fExplicitPrevNext);

    // align SuggestedShape to the shape model, put result in Shape

    bool fFinalIter = (iter == Model.AsmLevs[iLev].nMaxSearchIters - 1);
    Shape = ConformShapeToModel(b,
                                SuggestedShape, Model, iLev, fFinalIter);

    iter++;
    }
}

//-----------------------------------------------------------------------------
// Accumulate the results of the current search (in Shape) into the
// combined results for all searches (in CombinedShape).
// This basically copies Shape to CombinedShape but has to deal with
// the fact that these shapes may be different sizes.

static void
CombineSearchResults (
    SHAPE &CombinedShape,   // io: best shape formed by combining all searches
    int iModel,             // in
    const SHAPE &Shape)     // in: results of current search
{
if (iModel == 0)
    CombinedShape = Shape;
else
    {
    unsigned nPoints = CombinedShape.nrows();

    if (Shape.nrows() < nPoints)
        Err("the number of points %d in the 2nd model must be greater\n"
            "       than or equal to the number of points %d in the 1st model",
            Shape.nrows(), nPoints);

    for (unsigned iPoint = 0; iPoint < nPoints; iPoint++)
        {
        CombinedShape(iPoint, VX) = Shape(iPoint, VX);
        CombinedShape(iPoint, VY) = Shape(iPoint, VY);
        }
    }
}

//-----------------------------------------------------------------------------
// Returns the landmark positions in CombinedShape.
// If unsuccessful, returned CombinedShape.nrows() will be 0.
// This can only happen if the face detector fails (a message will be printed).

SHAPE                        // results returned as a SHAPE
AsmSearch (
    SHAPE &StartShape,       // out: start shape returned in here
    DET_PARAMS &DetParams,   // out: face detector parameters
    double &MeanTime,        // out: mean time per image (face det failed excluded)
    const RgbImage &RgbImg,  // in: find face features in this image
    const char sImage[],     // in: file path for RgbImg, for err msgs
    const bool fRowley,      // in: true to use Rowley detector, else VJ
    const char sConfFile0[], // in: 1st config filename
    const char sConfFile1[], // in: 2nd config filename, "" if none
    const char sDataDir[],   // in: data directory
    const char sShapeFile[], // in: if not NULL then use face detector in here
    bool fIssueWarnings)     // in: true to issue warnings if needed
{
static ASM_MODEL Models[NMODELS_MAX];
const int nInitializedModels = nInitAsmModels(Models, sConfFile0, sConfFile1);

// nModels will be less than nInitializedModels if we previously
// called nInitAsmModels with two models but now are not supplying sConfFile1.
// This works because nInitAsmModels only actually initializes the models once,
// and thereafter keeps track of how many models are initialized.
const int nModels = (nInitializedModels==2 && sConfFile1 && sConfFile1[0])? 2: 1;

static double TotalDetTime, TotalAsmTime;
static int nTotalImages; nTotalImages++;
static int nGoodImages;
clock_t StartTime = clock();    // note that this excludes time in nInitAsmModels

SHAPE Shape;

// CombinedShape is created by combining shapes from each model search.
// It has the same number of points as Models[0].FileMeanShape, so if
// Models[1] has extra points they are discarded after the Models[1] search.

SHAPE CombinedShape;

Image Img;
ConvertRgbImageToGray(Img, RgbImg);

unsigned DetAttr = FA_ViolaJones;   // specifies which face detector to use
SHAPE DetAv = Models[0].VjAv;
if (fRowley)
    {
    DetAttr = FA_Rowley;
    DetAv = Models[0].RowleyAv;
    }
if (fGetStartShape(StartShape, DetParams,
                   sImage, Models[0].FileMeanShape,
                   DetAttr, DetAv, sShapeFile, sDataDir,
                   CONF_fStasmSkipIfNotInShapeFile, fIssueWarnings))
    {
    TotalDetTime += double(clock() - StartTime) / CLOCKS_PER_SEC;
    StartTime = clock();
    nGoodImages++;
    Shape = StartShape;

    for (int iModel = 0; iModel < nModels; iModel++)
        {
        ASM_MODEL *pModel = &Models[iModel];
        if (iModel != 0)
            GetStartShapeFromPreviousSearch(Shape,
                CombinedShape, pModel->FileMeanShape);

        // Scale Shape and Img, so the face width is nStandardFaceWidth,
        // using the start shape to approximate the face width.

        double ImageScale = pModel->nStandardFaceWidth / xShapeExtent(Shape);
        SHAPE Shape1(Shape * ImageScale);   // working shape
        Image Img1(Img);                    // working Img
        int nNewWidth  = iround(Img1.width * ImageScale);
        int nNewHeight = iround(Img1.height * ImageScale);
        ScaleImage(Img1, nNewWidth, nNewHeight, IM_BILINEAR);

        // dimKeep is needed when this model has different number
        // of landmarks from previous model
        Shape1.dimKeep(pModel->nPoints, 2);
        int nStartLev = pModel->nStartLev;
        Shape1 /= GetPyrScale(nStartLev, pModel->PyrRatio);
        for (int iLev = nStartLev; iLev >= 0; iLev--)   // for each lev in image pyr
            {
            double PyrScale = GetPyrScale(iLev, pModel->PyrRatio);
            SEARCH_IMAGES SearchImgs;   // the images used during search
            SearchImgs.Img = Img1;      // SearchImgs.Img gets scaled to this pyr lev
            ReduceImage(SearchImgs.Img, PyrScale, pModel->PyrReduceMethod);
            InitGradsIfNeeded(SearchImgs.Grads,         // get SearchImgs.Grads
                pModel->AsmLevs[iLev].ProfSpecs, SearchImgs.Img, Shape1.nrows());

            AsmLevSearch(Shape1, SearchImgs, Models[iModel], iLev, gLandTab);

            if (iLev != 0) // use best shape from this iter as starting point for next
                Shape1 *= pModel->PyrRatio;

            ReleaseProcessor();         // give others a chance
            }
        if (nModels == 1)
            CombinedShape = Shape1;
        else
            CombineSearchResults(CombinedShape, iModel, Shape1);
        CombinedShape = CombinedShape / ImageScale; // descale back to original size
        }
    TotalAsmTime += double(clock() - StartTime) / CLOCKS_PER_SEC;
    }
MeanTime = (TotalDetTime + TotalAsmTime) / nGoodImages;

logprintf("\n[nTotalImages %d nGoodImages %d "
          "Mean times: FaceDet %.3f AsmSearch %.3f Both %.3f secs]\n",
    nTotalImages, nGoodImages,
    TotalDetTime / nGoodImages, TotalAsmTime / nGoodImages,
    MeanTime);

return CombinedShape;
}

//-----------------------------------------------------------------------------
// ASM search when files is already loaded

SHAPE                        // results returned as a SHAPE
AsmSearch2(
		   SHAPE &StartShape,       // out: start shape returned in here
		   DET_PARAMS &DetParams,   // out: face detector parameters
		   double &MeanTime,        // out: mean time per image (face det failed excluded)
            const int nInitializedModels, //the result of initiate !!!!!!
		  ASM_MODEL Models[],//initiated models
		   const RgbImage &RgbImg,  // in: find face features in this image
		   const char sImage[],     // in: file path for RgbImg, for err msgs
		   const bool fRowley,      // in: true to use Rowley detector, else VJ
		   const char sConfFile0[], // in: 1st config filename
		   const char sConfFile1[], // in: 2nd config filename, "" if none
		   const char sDataDir[],   // in: data directory
		   const char sShapeFile[], // in: if not NULL then use face detector in here
		   bool fIssueWarnings)     // in: true to issue warnings if needed
{
	
	// nModels will be less than nInitializedModels if we previously
	// called nInitAsmModels with two models but now are not supplying sConfFile1.
	// This works because nInitAsmModels only actually initializes the models once,
	// and thereafter keeps track of how many models are initialized.
	const int nModels = (nInitializedModels==2 && sConfFile1 && sConfFile1[0])? 2: 1;

	static double TotalDetTime, TotalAsmTime;
	static int nTotalImages; nTotalImages++;
	static int nGoodImages;
	clock_t StartTime = clock();    // note that this excludes time in nInitAsmModels

	SHAPE Shape;

	// CombinedShape is created by combining shapes from each model search.
	// It has the same number of points as Models[0].FileMeanShape, so if
	// Models[1] has extra points they are discarded after the Models[1] search.

	SHAPE CombinedShape;

	Image Img;
	ConvertRgbImageToGray(Img, RgbImg);

	unsigned DetAttr = FA_ViolaJones;   // specifies which face detector to use
	SHAPE DetAv = Models[0].VjAv;
	if (fRowley)
	{
		DetAttr = FA_Rowley;
		DetAv = Models[0].RowleyAv;
	}
	if (fGetStartShape(StartShape, DetParams,
		sImage, Models[0].FileMeanShape,
		DetAttr, DetAv, sShapeFile, sDataDir,
		CONF_fStasmSkipIfNotInShapeFile, fIssueWarnings))
	{
		TotalDetTime += double(clock() - StartTime) / CLOCKS_PER_SEC;
		StartTime = clock();
		nGoodImages++;
		Shape = StartShape;

		for (int iModel = 0; iModel < nModels; iModel++)
		{
			ASM_MODEL *pModel = &Models[iModel];
			if (iModel != 0)
				GetStartShapeFromPreviousSearch(Shape,
				CombinedShape, pModel->FileMeanShape);

			// Scale Shape and Img, so the face width is nStandardFaceWidth,
			// using the start shape to approximate the face width.

			double ImageScale = pModel->nStandardFaceWidth / xShapeExtent(Shape);
			SHAPE Shape1(Shape * ImageScale);   // working shape
			Image Img1(Img);                    // working Img
			int nNewWidth  = iround(Img1.width * ImageScale);
			int nNewHeight = iround(Img1.height * ImageScale);
			ScaleImage(Img1, nNewWidth, nNewHeight, IM_BILINEAR);

			// dimKeep is needed when this model has different number
			// of landmarks from previous model
			Shape1.dimKeep(pModel->nPoints, 2);
			int nStartLev = pModel->nStartLev;
			Shape1 /= GetPyrScale(nStartLev, pModel->PyrRatio);
			for (int iLev = nStartLev; iLev >= 0; iLev--)   // for each lev in image pyr
			{
				double PyrScale = GetPyrScale(iLev, pModel->PyrRatio);
				SEARCH_IMAGES SearchImgs;   // the images used during search
				SearchImgs.Img = Img1;      // SearchImgs.Img gets scaled to this pyr lev
				ReduceImage(SearchImgs.Img, PyrScale, pModel->PyrReduceMethod);
				InitGradsIfNeeded(SearchImgs.Grads,         // get SearchImgs.Grads
					pModel->AsmLevs[iLev].ProfSpecs, SearchImgs.Img, Shape1.nrows());

				AsmLevSearch(Shape1, SearchImgs, Models[iModel], iLev, gLandTab);

				if (iLev != 0) // use best shape from this iter as starting point for next
					Shape1 *= pModel->PyrRatio;

				ReleaseProcessor();         // give others a chance
			}
			if (nModels == 1)
				CombinedShape = Shape1;
			else
				CombineSearchResults(CombinedShape, iModel, Shape1);
			CombinedShape = CombinedShape / ImageScale; // descale back to original size
		}
		TotalAsmTime += double(clock() - StartTime) / CLOCKS_PER_SEC;
	}
	MeanTime = (TotalDetTime + TotalAsmTime) / nGoodImages;

	logprintf("\n[nTotalImages %d nGoodImages %d "
		"Mean times: FaceDet %.3f AsmSearch %.3f Both %.3f secs]\n",
		nTotalImages, nGoodImages,
		TotalDetTime / nGoodImages, TotalAsmTime / nGoodImages,
		MeanTime);

	return CombinedShape;
}